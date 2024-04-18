
class PlotGeneration:

    @staticmethod
    def created_plot(plots, ax, nums, coord_vehicle, names):

        for i in range(nums):
            plots += ax.plot(coord_vehicle[0, i, 0], coord_vehicle[0, i, 1], label=f'{names} {i}', linewidth=3.0)

    @staticmethod
    def update_animation(start_frame, frame, coord_vehicle, plots):

        for i, plot in enumerate(plots):
            plot.set_xdata(coord_vehicle[start_frame:frame, i, 0])
            plot.set_ydata(coord_vehicle[start_frame:frame, i, 1])
