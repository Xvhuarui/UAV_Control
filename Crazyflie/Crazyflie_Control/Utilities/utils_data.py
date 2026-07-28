import csv
import os

class DataLoggerIdeal:
    """飞行数据 CSV 保存类"""

    def __init__(self, output_dir):
        # 设置默认的保存文件夹，如果不存在则自动创建
        self.output_dir = output_dir
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)

    def save_to_csv(self, data_dict, filename):
        """
        将包含 NumPy 数组的字典保存为 CSV 文件
        :param data_dict: 你的 plot_data 字典
        :param filename: 保存的文件名
        """
        filepath = os.path.join(self.output_dir, filename)

        # 提取所有的列名 (字典的键)
        headers = list(data_dict.keys())

        # 获取数据的行数 (以时间列的长度为准)
        num_rows = len(data_dict['time'])

        print(f"\n正在将数据保存为 CSV 文件，共 {num_rows} 行，{len(headers)} 列...\n")

        try:
            # 打开文件准备写入
            with open(filepath, mode='w', newline='', encoding='utf-8') as file:
                writer = csv.writer(file)

                # 1. 写入表头 (第一行)
                writer.writerow(headers)

                # 2. 逐行写入数据
                for i in range(num_rows):
                    # 把字典里每个键对应的第 i 个元素提取出来，拼成一行
                    row_data = [data_dict[key][i] for key in headers]
                    writer.writerow(row_data)

            print(f"飞行数据成功保存至: {filepath}\n")

        except Exception as e:
            print(f"保存 CSV 文件时出错: {e}")
