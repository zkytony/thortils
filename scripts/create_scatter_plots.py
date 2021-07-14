import argparse
from thortils.scene import SceneDataset


def create_scatter_plots(rootdir, outdir):
    """Plot scatter plots for scenes."""
    dataset = SceneDataset.load(rootdir)
    dataset.plot_scatter_plots(outdir)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Build scene dataset.')
    parser.add_argument('rootdir', type=str, help='Root directory of dataset')
    parser.add_argument('outdir', type=str, help='Directory where output scatterplots are saved')
    args = parser.parse_args()
    confirm = input("Save scatter plots under {}? [y] ".format(args.outdir))
    if confirm.lower().startswith("y"):
        create_scatter_plots(args.rootdir, args.outdir)
