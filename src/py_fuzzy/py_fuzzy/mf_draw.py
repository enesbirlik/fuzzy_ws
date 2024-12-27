import numpy as np
import matplotlib.pyplot as plt
import skfuzzy as fuzz

def visualize_membership_functions():
    # Evren tanımları
    angle_universe = np.linspace(-15, 15, 300)
    angle_vel_universe = np.linspace(-150, 150, 300)
    effort_universe = np.linspace(-10, 10, 300)

    # Açı üyelik fonksiyonları
    angle_nvvb = fuzz.trimf(angle_universe, [-15, -15, -12])
    angle_nvb = fuzz.trimf(angle_universe, [-13, -10, -7])
    angle_nb = fuzz.trimf(angle_universe, [-8, -6, -4])
    angle_nm = fuzz.trimf(angle_universe, [-5, -3.5, -2])
    angle_ns = fuzz.trimf(angle_universe, [-3, -1.5, -0.5])
    angle_z = fuzz.trimf(angle_universe, [-1, 0, 1])
    angle_ps = fuzz.trimf(angle_universe, [0.5, 1.5, 3])
    angle_pm = fuzz.trimf(angle_universe, [2, 3.5, 5])
    angle_pb = fuzz.trimf(angle_universe, [4, 6, 8])
    angle_pvb = fuzz.trimf(angle_universe, [7, 10, 13])
    angle_pvvb = fuzz.trimf(angle_universe, [12, 15, 15])

    # Açısal hız üyelik fonksiyonları
    vel_nvvb = fuzz.trimf(angle_vel_universe, [-150, -150, -120])
    vel_nvb = fuzz.trimf(angle_vel_universe, [-130, -100, -70])
    vel_nb = fuzz.trimf(angle_vel_universe, [-80, -60, -40])
    vel_nm = fuzz.trimf(angle_vel_universe, [-50, -30, -15])
    vel_ns = fuzz.trimf(angle_vel_universe, [-20, -10, -3])
    vel_z = fuzz.trimf(angle_vel_universe, [-5, 0, 5])
    vel_ps = fuzz.trimf(angle_vel_universe, [3, 10, 20])
    vel_pm = fuzz.trimf(angle_vel_universe, [15, 30, 50])
    vel_pb = fuzz.trimf(angle_vel_universe, [40, 60, 80])
    vel_pvb = fuzz.trimf(angle_vel_universe, [70, 100, 130])
    vel_pvvb = fuzz.trimf(angle_vel_universe, [120, 150, 150])

    # Effort üyelik fonksiyonları
    effort_nvvb = fuzz.trimf(effort_universe, [-10, -10, -8])
    effort_nvb = fuzz.trimf(effort_universe, [-9, -7, -5])
    effort_nb = fuzz.trimf(effort_universe, [-6, -4, -2.5])
    effort_nm = fuzz.trimf(effort_universe, [-3, -2, -1])
    effort_ns = fuzz.trimf(effort_universe, [-1.5, -0.75, -0.25])
    effort_z = fuzz.trimf(effort_universe, [-0.5, 0, 0.5])
    effort_ps = fuzz.trimf(effort_universe, [0.25, 0.75, 1.5])
    effort_pm = fuzz.trimf(effort_universe, [1, 2, 3])
    effort_pb = fuzz.trimf(effort_universe, [2.5, 4, 6])
    effort_pvb = fuzz.trimf(effort_universe, [5, 7, 9])
    effort_pvvb = fuzz.trimf(effort_universe, [8, 10, 10])

    # Açı grafiği
    plt.figure(figsize=(12, 4))
    plt.plot(angle_universe, angle_nvvb, 'r', linewidth=1.5, label='NVVB')
    plt.plot(angle_universe, angle_nvb, 'b', linewidth=1.5, label='NVB')
    plt.plot(angle_universe, angle_nb, 'g', linewidth=1.5, label='NB')
    plt.plot(angle_universe, angle_nm, 'y', linewidth=1.5, label='NM')
    plt.plot(angle_universe, angle_ns, 'c', linewidth=1.5, label='NS')
    plt.plot(angle_universe, angle_z, 'm', linewidth=1.5, label='Z')
    plt.plot(angle_universe, angle_ps, 'k', linewidth=1.5, label='PS')
    plt.plot(angle_universe, angle_pm, 'orange', linewidth=1.5, label='PM')
    plt.plot(angle_universe, angle_pb, 'purple', linewidth=1.5, label='PB')
    plt.plot(angle_universe, angle_pvb, 'brown', linewidth=1.5, label='PVB')
    plt.plot(angle_universe, angle_pvvb, 'pink', linewidth=1.5, label='PVVB')
    plt.title('Angle Membership Functions')
    plt.xlabel('Angle (degrees)')
    plt.ylabel('Membership Degree')
    plt.grid(True)
    plt.axhline(y=0, color='k', linestyle='-')
    plt.axvline(x=0, color='k', linestyle='-')
    plt.xlim(-15, 15)  # X ekseni sınırları
    plt.ylim(-0.1, 1.1)  # Y ekseni sınırları
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.tight_layout()  # Grafiğin düzgün görünmesi için
    plt.show()

    # Açısal hız grafiği
    plt.figure(figsize=(12, 4))
    plt.plot(angle_vel_universe, vel_nvvb, 'r', linewidth=1.5, label='NVVB')
    plt.plot(angle_vel_universe, vel_nvb, 'b', linewidth=1.5, label='NVB')
    plt.plot(angle_vel_universe, vel_nb, 'g', linewidth=1.5, label='NB')
    plt.plot(angle_vel_universe, vel_nm, 'y', linewidth=1.5, label='NM')
    plt.plot(angle_vel_universe, vel_ns, 'c', linewidth=1.5, label='NS')
    plt.plot(angle_vel_universe, vel_z, 'm', linewidth=1.5, label='Z')
    plt.plot(angle_vel_universe, vel_ps, 'k', linewidth=1.5, label='PS')
    plt.plot(angle_vel_universe, vel_pm, 'orange', linewidth=1.5, label='PM')
    plt.plot(angle_vel_universe, vel_pb, 'purple', linewidth=1.5, label='PB')
    plt.plot(angle_vel_universe, vel_pvb, 'brown', linewidth=1.5, label='PVB')
    plt.plot(angle_vel_universe, vel_pvvb, 'pink', linewidth=1.5, label='PVVB')
    plt.title('Angular Velocity Membership Functions')
    plt.xlabel('Angular Velocity (degrees/second)')
    plt.ylabel('Membership Degree')
    plt.grid(True)
    plt.axhline(y=0, color='k', linestyle='-')
    plt.axvline(x=0, color='k', linestyle='-')
    plt.xlim(-150, 150)  # X ekseni sınırları
    plt.ylim(-0.1, 1.1)  # Y ekseni sınırları
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.tight_layout()
    plt.show()

    # Effort grafiği
    plt.figure(figsize=(12, 4))
    plt.plot(effort_universe, effort_nvvb, 'r', linewidth=1.5, label='NVVB')
    plt.plot(effort_universe, effort_nvb, 'b', linewidth=1.5, label='NVB')
    plt.plot(effort_universe, effort_nb, 'g', linewidth=1.5, label='NB')
    plt.plot(effort_universe, effort_nm, 'y', linewidth=1.5, label='NM')
    plt.plot(effort_universe, effort_ns, 'c', linewidth=1.5, label='NS')
    plt.plot(effort_universe, effort_z, 'm', linewidth=1.5, label='Z')
    plt.plot(effort_universe, effort_ps, 'k', linewidth=1.5, label='PS')
    plt.plot(effort_universe, effort_pm, 'orange', linewidth=1.5, label='PM')
    plt.plot(effort_universe, effort_pb, 'purple', linewidth=1.5, label='PB')
    plt.plot(effort_universe, effort_pvb, 'brown', linewidth=1.5, label='PVB')
    plt.plot(effort_universe, effort_pvvb, 'pink', linewidth=1.5, label='PVVB')
    plt.title('Effort Membership Functions')
    plt.xlabel('Effort (units)')
    plt.ylabel('Membership Degree')
    plt.grid(True)
    plt.axhline(y=0, color='k', linestyle='-')
    plt.axvline(x=0, color='k', linestyle='-')
    plt.xlim(-10, 10)  # X ekseni sınırları
    plt.ylim(-0.1, 1.1)  # Y ekseni sınırları
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    visualize_membership_functions()
