import matplotlib.pyplot as plt
import pandas as pd


def main():
    sim_df = pd.read_csv('ft_sensor_swing_sim.csv')
    real_df = pd.read_csv('ft_sensor_swing_real.csv')
    real_swing_start_idx = 1400

    # sim_df = pd.read_csv('ft_sensor_peel_off_sim.csv')
    # real_df = pd.read_csv('ft_sensor_peel_off_real.csv')	
    # real_swing_start_idx = 1125

    real_cut_df = real_df.iloc[real_swing_start_idx:]

    plt.figure()

    plt.plot(sim_df['time'], sim_df[['Fz_sim']].rolling(10).mean())

    # plt.gca().set_prop_cycle(None)

    # plt.plot((real_cut_df['time']-real_df['time'].iloc[real_swing_start_idx]), -real_cut_df[['Fz_real']].rolling(100).mean()) # - for the direction

    # plt.legend(bbox_to_anchor=[1, 1])

    plt.xlim([1, 5])
    plt.ylim([-15, 45])

    # plt.xlim([0, 0.4])

    plt.xlabel('Time (s)')
    plt.ylabel('Forces (N)')
    # plt.legend(['Fx_sim', 'Fy_sim', 'Fz_sim', 'Fx_real', 'Fy_real', 'Fz_real'], loc='center', bbox_to_anchor=[0.85, 0.6])
    plt.show()


if __name__ == '__main__':
    main()