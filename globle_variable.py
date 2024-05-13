import matplotlib.pyplot as plt

def create_ax():
    fig_size = [14, 9]
    x_lim = [-10, 10]
    y_lim = [0, 20]

    fig, ax = plt.subplots(figsize=fig_size)
    ax.set_xlim(x_lim)
    ax.set_ylim(y_lim)

    ax.set_visible(False)
    # 关闭无用的 fig 对象来释放资源
    plt.close(fig)
    return fig, ax

fig, ax = create_ax()