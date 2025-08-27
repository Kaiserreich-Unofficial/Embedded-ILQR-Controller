import serial
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import re

# ---------------------- 串口配置 ----------------------
SERIAL_PORT = 'COM6'  # Windows 示例，Linux 可改为 '/dev/ttyUSB0'
BAUD_RATE = 115200

# ---------------------- 数据存储 ----------------------
times = []
x_data, y_data, psi_data = [], [], []
u_data, v_data, r_data = [], [], []

# ---------------------- 正则解析 ----------------------
line_re = re.compile(
    r't=(?P<t>[0-9\.]+)\s+x=(?P<x>[0-9\.\-]+)\s+y=(?P<y>[0-9\.\-]+)\s+psi=(?P<psi>[0-9\.\-]+)\s+u=(?P<u>[0-9\.\-]+)\s+v=(?P<v>[0-9\.\-]+)\s+r=(?P<r>[0-9\.\-]+)\s+solve_time\(ms\)=(?P<solve>[0-9\.]+)'
)

# ---------------------- Matplotlib 图形设置 ----------------------
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
plt.subplots_adjust(wspace=0.3)

# 左图：轨迹
line_traj, = ax1.plot([], [], 'b-o', markersize=4)
ax1.set_xlabel('x (m)')
ax1.set_ylabel('y (m)')
ax1.set_title('USV Trajectory')
ax1.grid(True)
ax1.axis('equal')
# ---------------------- 在左图右上角显示 solve_time ----------------------
solve_text = ax1.text(0.95, 0.95, '', transform=ax1.transAxes,
                      ha='right', va='top', fontsize=12, bbox=dict(facecolor='white', alpha=0.7))
# 右图：u v r
line_u, = ax2.plot([], [], 'r-', label='u')
line_v, = ax2.plot([], [], 'g-', label='v')
line_r, = ax2.plot([], [], 'b-', label='r')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Velocity / Yaw Rate')
ax2.set_title('USV Velocity & Yaw Rate')
ax2.grid(True)
ax2.legend()

# ---------------------- 串口初始化 ----------------------
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)  # 等待串口稳定

# ---------------------- 更新函数 ----------------------


def update(frame):
    # 每次只读一行
    if ser.in_waiting:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        m = line_re.match(line)
        if m:
            t = float(m.group('t'))
            x = float(m.group('x'))
            y = float(m.group('y'))
            psi = float(m.group('psi'))
            u = float(m.group('u'))
            v = float(m.group('v'))
            r = float(m.group('r'))
            solve_time = float(m.group('solve'))

            times.append(t)
            x_data.append(x)
            y_data.append(y)
            psi_data.append(psi)
            u_data.append(u)
            v_data.append(v)
            r_data.append(r)

            # 更新轨迹图
            line_traj.set_data(x_data, y_data)
            ax1.relim()
            ax1.autoscale_view()

            # 更新 u v r 图
            line_u.set_data(times, u_data)
            line_v.set_data(times, v_data)
            line_r.set_data(times, r_data)
            ax2.relim()
            ax2.autoscale_view()

            # 更新右上角 solve_time
            solve_text.set_text(f'solve_time={solve_time:.1f} ms')

    return line_traj, line_u, line_v, line_r, solve_text


# ---------------------- 动画 ----------------------
ani = FuncAnimation(fig, update, interval=100,
                    cache_frame_data=False)  # 每10ms刷新
plt.show()
