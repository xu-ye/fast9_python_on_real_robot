import json
import numpy as np


class NumpyArrayEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)


array1=np.zeros((4,5))
data={'joint_angles':array1}
data_json = json.dumps(data, cls=NumpyArrayEncoder)
print(f'data_json:{data_json}')

# 写入json文件
with open('array_json.json', 'w') as f:
    json.dump(data, f, cls=NumpyArrayEncoder)

# 直接从json类型转化为字典
data_loads = json.loads(data_json)
print(f'data_loads:{data_loads}')

# 从json文件读取numpy数组
with open('array_json.json', 'r') as f:
    data_read = json.load(f)
    arr1_read = np.asarray(data_read['joint_angles'])


print(f'arr1_read:{arr1_read}\n')





