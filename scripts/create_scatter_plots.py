########## Used to create scatter plots
def create_scatter_plots():
    dataset = SceneDataset.load("./scenes")
    dataset.plot_scatter_plots("./scene_scatter_plots")

if __name__ == "__main__":
    create_scatter_plots()
