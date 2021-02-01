import os

for i in range(10):
    fo = open("test.txt", "w")
    str = f"number: {i}\n"
    fo.write(str)
    fo.close()
    import pdb; pdb.set_trace()

# 关闭文件