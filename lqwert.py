import os
import re
#
#
# # 替换原则：
# # * '区域车数' -> 'Area Vehicle Count'
# # * '数量' -> 'count'
# # * '行车方向' -> 'Vehicle Direction'
# # * '方向' -> 'direction'
# # * '进入车道' -> 'Enter Lane'
# # * '是否进入' -> 'entered'
# # * '驶出车道' -> 'Exit Lane'
# # * '是否驶出' -> 'exited'
#
# def replace_chinese(content):
#     # content = re.sub(r'区域车数', 'Area Vehicle Count', content)
#     # content = re.sub(r'数量', 'count', content)
#     # content = re.sub(r'行车方向', 'Vehicle Direction', content)
#     # content = re.sub(r'方向', 'direction', content)
#     # content = re.sub(r'进入车道', 'Enter Lane', content)
#     # content = re.sub(r'进入车位', 'Enter Area', content)
#     # content = re.sub(r'是否进入', '\'entere\'d', content)
#     # content = re.sub(r'驶出车道', 'Exit Lane', content)
#     # content = re.sub(r'是否驶出', 'exited', content)
#     content = re.sub(r'\'entere\'dd', '\'entered\'', content)
#     content = re.sub(r'\'exited\'d', '\'exitedd\'', content)
#     return content
#
#
# def process_label_files(folder_path):
#     count = 1
#     for filename in os.listdir(folder_path):
#         if filename.endswith('.txt'):
#             file_path = os.path.join(folder_path, filename)
#
#             # 读取文件内容
#             with open(file_path, 'r', encoding='utf-8') as file:
#                 content = file.read()
#
#             # 替换汉字为英文
#             updated_content = replace_chinese(content)
#
#             # 将更新的内容写回文件
#             with open(file_path, 'w', encoding='utf-8') as file:
#                 file.write(updated_content)
#         count += 1
#         print(count)
#
#
# # 修改为您的文件夹路径
# folder_path = "G:\jieshun\project_code\map\map\example_4_15\label"
# process_label_files(folder_path)
import os


def process_label_file(folder_path):
    count = 1
    for filename in os.listdir(folder_path):
        if filename.endswith('.txt'):
            file_path = os.path.join(folder_path, filename)

            with open(file_path, 'r', encoding='utf-8') as file:
                content = file.read()

            content = content.replace("'", '"')
            content = "[" + content.replace("\n", ",") + "]"
            content = re.sub(r',]', ']', content)
            content = re.sub(r',]', ']', content)


            writ_file = file_path
            writ_file = re.sub('label', 'json', writ_file)
            writ_file = re.sub('txt', 'json', writ_file)

            # 将更新的内容写回文件
            with open(writ_file, 'w', encoding='utf-8') as file:
                file.write(content)
            print(f"\r{count}", end=' ')
            count += 1

    return content


# 指定文件路径
file_path = "G:\\jieshun\\project_code\\map\map\\example_4_15\\label"
content = process_label_file(file_path)

# with open("processed_file.json", "w", encoding="utf-8") as file:
#     file.write(content)

