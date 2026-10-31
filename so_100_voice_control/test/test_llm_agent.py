"""LlmAgent: tools come from OpenAPI, the tool-call loop executes them,
blocked/dry-run rules hold. Ollama and the dashboard are faked."""
import pytest

from so_100_voice_control import config, llm_agent
from so_100_voice_control.llm_agent import LlmAgent, build_tools, started_program

OPENAPI = {
    'paths': {
        '/api/targets': {
            'get': {'summary': 'List saved targets'},
            'post': {'summary': 'Record', 'requestBody': {'content': {'application/json': {
                'schema': {'$ref': '#/components/schemas/Rec'}}}}},
        },
        '/api/targets/{target_name}': {
            'delete': {'summary': 'Delete', 'parameters': [
                {'name': 'target_name', 'in': 'path', 'required': True, 'schema': {'type': 'string'}}]},
        },
        '/api/programs/{program_name}/run': {
            'post': {'summary': 'Run', 'parameters': [
                {'name': 'program_name', 'in': 'path', 'required': True, 'schema': {'type': 'string'}}]},
        },
        '/api/program/pause': {'post': {'summary': 'Not supported'}},
    },
    'components': {'schemas': {'Rec': {
        'type': 'object', 'required': ['name'],
        'properties': {'name': {'type': 'string', 'title': 'Name'},
                       'index': {'anyOf': [{'type': 'integer'}, {'type': 'null'}]}}}}},
}


class FakeClient:
    def __init__(self):
        self.calls = []

    def get_openapi(self):
        return OPENAPI

    def call(self, method, path, params=None, body=None, timeout=None):
        self.calls.append((method, path, body))
        return {'ok': True}


SENT = []


def script_ollama(monkeypatch, steps):
    """Each step is the assistant `message` the fake model returns next."""
    it = iter(steps)

    class Resp:
        def __init__(self, message):
            self._m = message

        def raise_for_status(self):
            pass

        def json(self):
            return {'message': self._m}

    SENT.clear()

    def post(*a, **k):
        SENT.append(k['json'])
        return Resp(next(it))

    monkeypatch.setattr(llm_agent.requests, 'post', post)


def tool_call(tool, **args):
    return {'role': 'assistant', 'content': '', 'tool_calls': [{'function': {'name': tool, 'arguments': args}}]}


def test_build_tools_from_openapi():
    tools = build_tools(OPENAPI)
    assert set(tools) == {'get_api_targets', 'post_api_targets', 'delete_api_targets_target_name',
                          'post_api_programs_program_name_run'}  # 'Not supported' skipped
    props = tools['post_api_targets'].schema['function']['parameters']['properties']
    assert props['name'] == {'type': 'string'} and props['index'] == {'type': 'integer'}
    assert tools['post_api_targets'].schema['function']['parameters']['required'] == ['name']


def test_agent_chains_calls_and_replies(monkeypatch):
    client = FakeClient()
    script_ollama(monkeypatch, [
        tool_call('post_api_targets', name='pick_start'),
        tool_call('post_api_programs_program_name_run', program_name='my prog'),
        {'role': 'assistant', 'content': 'Recorded and started.'},
    ])
    result = LlmAgent(client).run('record pick start then run my prog')
    assert result.reply == 'Recorded and started.'
    assert client.calls == [('POST', '/api/targets', {'name': 'pick_start'}),
                            ('POST', '/api/programs/my%20prog/run', None)]
    assert started_program(result.calls) == 'my prog'


def test_blocked_operation_is_refused(monkeypatch):
    monkeypatch.setattr(config, 'LLM_BLOCKED_OPS', 'DELETE /')
    client = FakeClient()
    script_ollama(monkeypatch, [
        tool_call('delete_api_targets_target_name', target_name='a'),
        {'role': 'assistant', 'content': 'Do that in the dashboard.'},
    ])
    LlmAgent(client).run('delete target a')
    assert client.calls == []


def test_dry_run_sends_reads_only(monkeypatch):
    client = FakeClient()
    script_ollama(monkeypatch, [
        tool_call('get_api_targets'),
        tool_call('post_api_targets', name='x'),
        {'role': 'assistant', 'content': 'ok'},
    ])
    LlmAgent(client, dry_run=True).run('x')
    assert client.calls == [('GET', '/api/targets', None)]


def test_step_cap(monkeypatch):
    monkeypatch.setattr(config, 'LLM_AGENT_MAX_STEPS', 3)
    script_ollama(monkeypatch, [tool_call('get_api_targets')] * 10)
    assert 'too many steps' in LlmAgent(FakeClient()).run('loop').reply


def test_real_dashboard_openapi_builds_tools():
    pytest.importorskip('so101_dashboard.api.server')
    from so101_dashboard.api.server import create_app
    tools = build_tools(create_app(None).openapi())
    assert 'post_api_targets' in tools and 'post_api_programs_program_name_elements' in tools


def test_null_args_are_dropped(monkeypatch):
    client = FakeClient()
    script_ollama(monkeypatch, [
        tool_call('post_api_targets', name='x', index=None),
        {'role': 'assistant', 'content': 'ok'},
    ])
    LlmAgent(client).run('x')
    assert client.calls == [('POST', '/api/targets', {'name': 'x'})]


def test_history_is_folded_into_user_text_not_sent_as_messages(monkeypatch):
    script_ollama(monkeypatch, [
        {'role': 'assistant', 'content': 'Opened.'},
        {'role': 'assistant', 'content': 'Closed.'},
    ])
    agent = LlmAgent(FakeClient())
    agent.run('open the gripper')
    agent.run('close it')
    second = SENT[1]
    assert [m['role'] for m in second['messages']] == ['system', 'user']
    assert 'open the gripper' in second['messages'][1]['content']
    assert second['messages'][1]['content'].endswith('Now: close it')
    assert second['keep_alive'] == -1


def test_warm_up_sends_one_request_and_never_raises(monkeypatch):
    script_ollama(monkeypatch, [{'role': 'assistant', 'content': ''}])
    LlmAgent(FakeClient()).warm_up()
    assert len(SENT) == 1 and SENT[0]['keep_alive'] == -1

    def boom(*a, **k):
        raise llm_agent.requests.exceptions.ConnectionError('down')
    monkeypatch.setattr(llm_agent.requests, 'post', boom)
    LlmAgent(FakeClient()).warm_up()
