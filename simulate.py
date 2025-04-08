import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# CSV 읽기
robot_df = pd.read_csv("F:/OptimalControl/PersonalData/mobile_robot_project/output/robot_trajectory.csv")
env_df = pd.read_csv("F:/OptimalControl/PersonalData/mobile_robot_project/output/environment.csv")
# 열: time_step, step_idx, x, y, cur_x, cur_y, goal_x, goal_y

# 2) time_step 목록 추출 (오름차순)
time_steps = sorted(robot_df["time_step"].unique())

# 3) matplotlib figure 준비
fig, ax = plt.subplots()

# 궤적을 그릴 선(line) 객체. 여기선 모든 궤적점들을 하나의 line으로 그리는 예시.
# 필요하다면 step_idx별로 다른 라인을 쓸 수도 있음.
traj_line, = ax.plot([], [], 'b-', label="Trajectory")

# 현재 위치 (로봇) 마커, 목표점(Goal) 마커
robot_marker, = ax.plot([], [], 'ro', label="Robot")
goal_marker,  = ax.plot([], [], 'g*', label="Goal")
obs_marker, = ax.plot([], [], 'ko', label="Obstacle")
# (옵션) 좌표 범위 설정
ax.set_xlim(-1, 3)  
ax.set_ylim(-1, 3)
ax.legend()
ax.set_title("Trajectory Animation by time_step")
ax.grid(True)

# 4) 초기화 함수
def init():
    # 아무것도 표시되지 않은 상태로 시작
    traj_line.set_data([], [])
    robot_marker.set_data([], [])
    goal_marker.set_data([], [])
    obs_marker.set_data([], [])
    return traj_line, robot_marker, goal_marker, obs_marker

# 5) 매 frame(= time_step)마다 실행되는 업데이트 함수
def update(frame):
    t = time_steps[frame]
    
    # 이 time_step에 해당하는 행들만 뽑아온다
    sub = robot_df[robot_df["time_step"] == t]

    # (a) 모든 궤적점(step_idx 별) 좌표 (x, y)
    xs = sub["x"]
    ys = sub["y"]
    traj_line.set_data(xs, ys)

    # (b) 이 time_step의 현재 로봇 위치 (cur_x, cur_y)
    #     같은 time_step 행들엔 cur_x,cur_y가 모두 같으므로 첫 번째 행만 꺼내도 됨
    cur_x = sub["cur_x"].iloc[0]
    cur_y = sub["cur_y"].iloc[0]
    robot_marker.set_data(cur_x, cur_y)

    # (c) 목표점 (goal_x, goal_y)
    goal_x = env_df["goal_x"].iloc[0]
    goal_y = env_df["goal_y"].iloc[0]
    goal_marker.set_data(goal_x, goal_y)

    obs_x = env_df["obs_x"]
    obs_y = env_df["obs_y"]
    obs_marker.set_data(obs_x, obs_y)

    return traj_line, robot_marker, goal_marker

# 6) FuncAnimation 생성
ani = FuncAnimation(fig, update,
                    frames=len(time_steps),
                    init_func=init,
                    interval=1,   # ms, 각 프레임 간 간격
                    blit=False)
ani.save("simulation3.gif", writer='pillow', fps=100)
#plt.show()