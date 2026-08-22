from setuptools import find_packages, setup

package_name = 'so_100_voice_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        # sounddevice, scipy, faster-whisper, and requests are not
        # available as rosdep/apt keys for Humble; install them with pip
        # (see README) before running, same as so101_dashboard does for
        # its own non-rosdep Python deps.
        'sounddevice',
        'scipy',
        'numpy',
        'faster-whisper',
        'requests',
    ],
    zip_safe=True,
    maintainer='revanth',
    maintainer_email='revanthsriraj7@gmail.com',
    description='Voice control for the SO-101 dashboard: mic -> faster-whisper -> command parser -> dashboard REST API.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
        # Optional: better-than-espeak local neural TTS (see tts.py).
        # speak() falls back to espeak/espeak-ng cleanly without this.
        'tts': [
            'piper-tts',
        ],
    },
    entry_points={
        'console_scripts': [
            'voice_control = so_100_voice_control.voice_control_node:main',
        ],
    },
)
