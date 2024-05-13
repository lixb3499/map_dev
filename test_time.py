import time
from Map import Border, Lane

my_list = range(100)
total = len(my_list)

for i, item in enumerate(my_list):
    # time.sleep(0.1)  # 模拟循环中的操作，仅作示例
    progress_percentage = (i + 1) * 100 / total

    # 使用 '\r' 返回行首，'end=""' 防止换行
    print(f"\rProcessing: {progress_percentage:.1f}% complete", end="")

# 完成循环后打印一个换行符，以便在循环结束后可以继续正常打印
print()

border = Border((1,0), (1,1))
print(border.axis)

lane = Lane(0, (0,4), (4, 0))
a = lane.intersect((1,3), (1,5))
print("a=", a)



##################################################################################3
import yaml

def load_config_from_yaml(yaml_file):
    with open(yaml_file, 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)
    return config

# 读取配置文件
config_file = 'config.yaml'
config = load_config_from_yaml(config_file)

# 在程序中访问参数值
print(config['data_file'])
print(config['frame_rate'])
print(config['areas'])