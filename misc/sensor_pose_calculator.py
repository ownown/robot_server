import math
import matplotlib.pyplot as plt
from typing import List, Tuple

def sensor_angles(number_of_sensors: int, offset: float) -> List[float]:
    sensor_angle: float = (2*math.pi) / number_of_sensors
    angles: float = [x * sensor_angle + offset for x in range(0, number_of_sensors)]
    return list(map(lambda x: ((2*math.pi) + x) if x < 0 else x, angles))

def sensor_pose(theta_k: float, x_0: float, y_0: float, x_offset: float = 0.0, y_offset: float = 0.0) -> Tuple[float, float, float]:
    x: float = (x_0 * math.cos(theta_k)) - (y_0 * math.sin(theta_k)) - x_offset
    y: float = (x_0 * math.sin(theta_k)) + (y_0 * math.cos(theta_k)) - y_offset

    return (x, y, theta_k)

def distance_from_origin(x: float,y: float) -> float:
    return math.sqrt(x*x + y*y)

def print_table(poses: List[Tuple[float, float, float]]) -> None:
    print('{:<9} {:<9} {:<6} {}'.format('X,', 'Y,', 'Deg,', 'Distance from origin'))
    [print(f'{x: 8.3f}, {y: 8.3f}, {math.degrees(t):5.1f}, {distance_from_origin(x,y):5.1f}') for x,y,t in poses]

def print_chart(poses: List[Tuple[float, float, float]]) -> None:
    distances: List[float] = [distance_from_origin(x,y) for x,y,_ in poses]
    max_val: float = max(distances)
    plt.plot([-max_val,max_val],[0,0], color='black', linewidth=0.2, zorder=0)
    plt.plot([0,0], [-max_val,max_val], color='black', linewidth=0.2, zorder=0)
    for i in range(len(poses)):
        plt.plot([0,poses[i][0]],[0,poses[i][1]], color='green', linestyle='--', linewidth=0.3, zorder=0)
        plt.plot([poses[i][0],poses[(i+1) % len(poses)][0]],[poses[i][1],poses[(i+1) % len(poses)][1]], color='blue', linestyle='--', linewidth=0.3, zorder=0)

    plt.scatter(0,0, color='red')

    x,y,_ = list(map(list, zip(*poses)))
    plt.scatter(x,y, color='blue')
    plt.show()    

def save_to_csv(poses: List[Tuple[float, float, float]], filename: str) -> None:
    with open(filename, 'w') as f:
        # f.write('x,y,theta\n') # Not really needed
        for x,y,t in poses:
            f.write(f'{x:.9f},{y:.9f},{t:.9f}\n')

if __name__ == '__main__':
    CSV_FILE: str = 'sensor_poses.csv'
    NUM_SENSORS: int = 8
    SENSOR_ANGLE_OFFSET: float =  - ((3 * math.pi) / 8)
    SENSOR_X_OFFSET: float = 23.4

    X_0: float = 112.8
    Y_0: float = 0.0

    angles: List[float] = sensor_angles(NUM_SENSORS, SENSOR_ANGLE_OFFSET)
    poses: List[Tuple[float, float, float]] = [sensor_pose(t_k, X_0, Y_0, SENSOR_X_OFFSET) for t_k in angles]
    
    save_to_csv(poses, CSV_FILE)
    # print_table(poses)
    # print_chart(poses)
