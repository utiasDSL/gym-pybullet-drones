
class PlotGeneration:

    @staticmethod
    def created_plot(plots, ax, nums, coord_vehicle, names, size):

        for i in range(nums):
            plots += ax.plot(coord_vehicle[0, i, 0], coord_vehicle[0, i, 1], label=f'{names} {i}', linewidth=size)

    @staticmethod
    def update_animation(start_frame, frame, coord_vehicle, plots):

        for i, plot in enumerate(plots):
            plot.set_xdata(coord_vehicle[start_frame:frame, i, 0])
            plot.set_ydata(coord_vehicle[start_frame:frame, i, 1])
