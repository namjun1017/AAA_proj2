from setuptools import find_packages, setup

package_name = 'order_details'

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
        'rclpy',                  # ROS 2 Python 클라이언트
        'std_msgs',               # String 등 표준 메시지
        
        # 커스텀 메시지 패키지
        'order_interfaces',       
        
        # 외부 Python 라이브러리
        'python-dotenv',
        'langchain-openai',       # ChatOpenAI 사용
        'langchain-core',         # PromptTemplate 및 runnables 사용
        'openai',                 # OpenAI Python Client (GPT- API 호출)
        
        # 정규표현식, JSON 등은 내장 라이브러리이므로 불필요
    ],
    zip_safe=True,
    maintainer='nj',
    maintainer_email='rlaskawns101@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'order_details = order_details.order_details:main',
        ],
    },
)
