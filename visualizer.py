import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


class DataAnalyzer:
    def __init__(self, input_locations, dims):
        self.x_lim = dims[0]
        self.y_lim = dims[1]
        self.df = pd.DataFrame()
        for file in input_locations:
            df = pd.read_csv(file)
            self.df = pd.concat([self.df, df])

        # self.df = pd.read_csv(input_location)
        self.df = self.df.drop(self.df[(self.df.mocap_x == 0) | (self.df.vision_x == 0)].index)
        self.avg_fps = 0
        self.avg_position = (0, 0)

    def export_to_csv(self, output_location):
        self.df.to_csv(output_location)

    def add_fps_to_df(self):
        timestamps = self.df['t'].to_numpy()
        fps = [0]

        for i in range(1, len(timestamps)):
            if timestamps[i] - timestamps[i-1] != 0:
                fps.append(1/(timestamps[i] - timestamps[i-1]))
            else:
                fps.append(fps[i-1])

        self.df.insert(len(self.df.columns), 'fps', fps)

    
    def visualize_fps_raw(self):
        if 'fps' not in self.df.columns:
            self.add_fps_to_df()

        fps = self.df['fps'].to_numpy()
        timesteps = np.linspace(0, fps.size - 1, fps.size)
        fig = plt.figure()
        ax = fig.add_subplot()
        plt.scatter(timesteps, fps, c='blue')
        ax.set_xlabel('frames')
        ax.set_ylabel('fps [1/s]')
        print(np.average(fps))
        plt.show()

    def visualize_axis_raw(self, axis):
        data = self.df[axis].to_numpy()
        timesteps = np.linspace(0, data.size - 1, data.size)
        fig = plt.figure()
        ax = fig.add_subplot()
        plt.scatter(timesteps, data, c='blue')
        ax.set_xlabel('timestep')
        ax.set_ylabel(axis)
        
        print(axis)
        print(f'Mean: {np.nanmean(data)}')
        print(f'Standard deviation: {np.nanstd(data)}')
        
        plt.show()

    def visualize_3d_pixels(self):
        x_vec = self.df['x'].to_numpy()
        y_vec = self.df['y'].to_numpy()
        z_vec = self.df['z'].to_numpy()

        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')

        for i in range(0, len(x_vec)):
            # swap y and z axis in visualisation
            ax.scatter(x_vec[i], z_vec[i], y_vec[i], c='blue')

        ax.set_xlabel('x [m]')
        ax.set_ylabel('z [m]')
        ax.set_zlabel('y [m]')

        z_lim = np.max(z_vec)
        ax.set_xlim((0, self.x_lim))
        ax.set_ylim((0, z_lim))
        ax.set_zlim((0, self.y_lim))

        plt.show()

    def visualize_3d_meters(self):
        x_vec = self.df['x'].to_numpy()
        y_vec = self.df['y'].to_numpy()
        z_vec = self.df['z'].to_numpy()

        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')

        for i in range(0, len(x_vec)):
            # swap y and z axis in visualisation
            ax.scatter(x_vec[i], z_vec[i], y_vec[i], c='blue')

        ax.set_xlabel('x [m]')
        ax.set_ylabel('z [m]')
        ax.set_zlabel('y [m]')

        x_lim = (np.min(x_vec), np.max(x_vec))
        y_lim = (np.min(z_vec), np.max(z_vec))
        z_lim = (np.min(y_vec), np.max(y_vec))

        ax.set_xlim((x_lim[0], x_lim[1]))
        ax.set_zlim((z_lim[0], z_lim[1]))
        ax.set_ylim((y_lim[0], y_lim[1]))

        plt.show()

    def visualize_2D_pixels(self):
        x_vec = self.df['x'].to_numpy()
        y_vec = self.df['y'].to_numpy()

        fig = plt.figure()
        ax = fig.add_subplot()

        for i in range(0, len(x_vec)):
            ax.scatter(x_vec[i], y_vec[i], c='blue')

        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')

        ax.set_xlim((0, self.x_lim))
        ax.set_ylim((0, self.y_lim))

        plt.show()




def analyse_axis(df, axis, plot=False):
    data = df[axis].to_numpy()
    timesteps = np.linspace(0, data.size - 1, data.size)
    if plot:
        fig = plt.figure()
        ax = fig.add_subplot()
        plt.scatter(timesteps, data, c='blue')
        ax.set_xlabel('timestep')
        ax.set_ylabel(axis)
    
    print(axis)
    print(f'Mean: {(mean := np.nanmean(data))}')
    print(f'Standard deviation: {(std := np.nanstd(data))}')

    return mean, std
    
    # plt.show()

def plot_vector(vec, y_label, x_vec=None, x_label=None):
    data = np.asarray(vec)
    timesteps = np.linspace(0, data.size - 1, data.size) if x_vec is None else x_vec
    x_label = '' if x_label is None else x_label
    fig = plt.figure()
    ax = fig.add_subplot()
    plt.scatter(timesteps, data, c='blue')
    # plt.plot(data, timesteps)
    ax.set_xlabel(x_label)
    ax.set_ylabel(y_label)
    min_val = np.abs(np.min(data))
    max_val = np.abs(np.max(data))
    if min_val > max_val:
        max_val = min_val

    plt.axhline(y=0.0, color='r')
    ax.set_ylim(-max_val - 0.05 * max_val, max_val + 0.05 * max_val)
    plt.show()

    
if __name__ == '__main__':
    vis = DataAnalyzer(
        [
        'logs/bottle_-1_0_05/bottle_static_1m.csv',
        'logs/bottle_-15_0_05/test.csv',
        'logs/bottle_-2_0_05/test.csv'
        ], (480, 640))
    vis.export_to_csv('testing_concat.csv')
    x_vals = [1.7 + i*0.5 for i in range(0,3)]
    y_val = 0.0
    z_val = 0.05
    tolerance = 0.05
    means = []
    stds = []
    axis = ['error_x', 'error_y', 'error_z']
    for val in x_vals:
        expr = abs(abs(vis.df.quad_x - vis.df.mocap_x) - val)
        
        
        data = expr.to_numpy(dtype='float64')
        idx = np.where(data < 0.05)
        data = data[idx]
        print(f'numpy data shape: {data.shape}')
        
        # timesteps = np.linspace(0, data.shape[0] - 1, data.shape[0])
        # fig = plt.figure()
        # ax = fig.add_subplot()
        # plt.scatter(timesteps, data, c='blue')
        # plt.show()


        temp_df = vis.df[expr < 0.05]
        print(f'dataframe shape: {temp_df.shape}')
        df_means = []
        df_stds = []
        for a in axis:
            print(val)
            mean, std = analyse_axis(temp_df, a)
            df_means.append(mean)
            df_stds.append(std)
        
        means.append(df_means)
        stds.append(df_stds)

    means_simplified = []
    for entry in means:
        axis_mean = np.mean(entry)
        means_simplified.append(axis_mean)


    plot_vector(means_simplified, 'Mean Error [m]', x_vals, 'Translation in x [m]')

    
