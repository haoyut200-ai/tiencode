import subprocess
import time

# 定义要按顺序运行的脚本列表
scripts = ["seg3.py", "seg4.py", "seg5.py", "seg6.py"]

for i, script in enumerate(scripts, start=1):
    # 给出信号指令
    print(f"========== 正在运行第 {i} 步: {script} ==========")
    
    # 运行对应的脚本
    # 假设您的环境变量中 python 命令为 'python'，如果是 python3 请将其替换为 'python3'
    subprocess.run(["python3", script])
    
    
    # 如果不是最后一个脚本，则等待2秒
    if i < len(scripts):
        print(f"等待 0.5 秒钟...\n")
        time.sleep(0.5)

print("========== 所有步骤运行完毕！ ==========")