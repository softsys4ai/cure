import os.path
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
from matplotlib.ticker import MaxNLocator
import matplotlib.cm as cm
import matplotlib.tri as mtri
from utils.eval import EvalMetrics
import warnings
import argparse
warnings.filterwarnings('ignore')

style.use('bmh')
plt.rcParams["font.family"] = "serif"

fig = plt.figure(figsize=(11,8))
fig.canvas.set_window_title('CURE Live Resuls')
ax1 = fig.add_subplot(2,2,1)
ax2 = fig.add_subplot(2,2,2)
ax3 = fig.add_subplot(2,2,3)
# ax4 = fig.add_subplot(2,2,4)
ax4 = fig.add_subplot(224, projection='3d')
ax4.view_init(elev=26, azim=-33)
cbar = None

parser = argparse.ArgumentParser(description='CURE Live Plot')
parser.add_argument('--hv_ref_f1', 
                    metavar='', 
                    required=True,
                    type=float,                                      
                    help='hypervolum reference for f1')    
parser.add_argument('--hv_ref_f2', 
                    metavar='', 
                    required=True,
                    type=float,                                      
                    help='hypervolum reference for f2') 
args = parser.parse_args() 

def live_result(i):
    global cbar
    if os.path.isfile("cure_log/AXMO_results.csv"):
        data = "cure_log/AXMO_results.csv"
        eval = EvalMetrics()
        n_iter, hv = eval.hypervolume(data=data,
                            f1="Energy",
                            f2="Positional_error",
                            f1_ref=args.hv_ref_f1,
                            f2_ref=args.hv_ref_f2,
                            )
        n_iter, energy, pose_error, safety, task_success, rns, hv, pareto = eval.read_data(data, "Energy", "Positional_error")
        efficiency = eval.success_rate(task_success)        
        ax1.clear()
        ax2.clear()
        ax3.clear()
        ax4.clear()

        ax1.plot(n_iter, hv.ewm(span=max(n_iter)).mean(),color="tab:orange")
        for i in range(len(n_iter)):
            if rns[i] >= 0.8:
                ax1.scatter(n_iter[i], hv.ewm(span=max(n_iter)).mean()[i], color="r", marker="*", s=80) 
            else:
                ax1.scatter(n_iter[i], hv.ewm(span=max(n_iter)).mean()[i],color="tab:orange", marker="o")
        ax1.set_title("Hypervolume")
        ax1.set_xlabel("Iteration")
        ax1.set_ylabel("Hypervoulme")
        # ax1.set_ylim(0.7,1)
        ax1.xaxis.set_major_locator(MaxNLocator(integer=True))


        ax2.step(pareto[0][0], pareto[1][0], color="tab:orange")
        # ax2.fill_between(range(15,41), 0.12, 0.18, color='grey', alpha=0.5, label="Threshold")
        ax2.set_title("Pareto front")
        ax2.set_xlabel("Energy (Wh)")
        ax2.set_ylabel(r"Pose error ($E_{dist}$)")
        ax2.xaxis.set_major_locator(MaxNLocator(integer=True))

        ax3.plot(n_iter, efficiency, color="tab:orange", marker="o", markeredgecolor='darkorange') 
        ax3.set_title("Efficiency")
        ax3.set_xlabel("Iteration")
        ax3.set_ylabel(r"$\eta$")
        # ax3.set_ylim(0,1.2)
        ax3.xaxis.set_major_locator(MaxNLocator(integer=True))
        
        ## For surface plot
        # try:
        #     ## For Triangulate
        #     # Triangulate the data
        #     triang = mtri.Triangulation(pose_error, energy)
        #     # Create a 3D surface plot
        #     # surf = ax4.plot_trisurf(triang, rns, cmap='plasma', edgecolor='k', linewidth=0, alpha=1, antialiased=True)
        #     contour = ax4.tricontourf(triang, rns, cmap='plasma', levels=500, alpha=0.5)
        #     # Overlay scatter plot for individual points with colors based on iteration
        #     sc = ax4.scatter(pose_error, energy, rns, c=n_iter, cmap='plasma',marker='')   
        #     ax4.set_title("Budget utilization")
        #     ax4.set_ylabel('Energy (Wh)')
        #     ax4.set_xlabel(r"Pose error ($E_{dist}$)")
        #     ax4.set_zlabel(r'$\mathcal{T}_{cr}$')   
        #     if cbar:
        #         cbar.remove()     
        #     cbar = plt.colorbar(sc, shrink=0.7, ax=ax4, pad=0.15)
        #     cbar.set_label('Iteration')  
        # except:
        #     sc = ax4.scatter(pose_error, energy, rns, s=30,  c=n_iter, cmap=cm.plasma)
        #     # Update colorbar properties  
        #     ax4.set_title("Budget utilization")
        #     ax4.set_ylabel("Energy (Wh)")
        #     ax4.set_xlabel(r"Pose error ($E_{dist}$)")  
        #     ax4.set_zlabel(r"$\mathcal{T}_{cr}$")
        #     plt.tick_params(labelsize=8, direction="in")
        #     ax4.xaxis.set_major_locator(MaxNLocator(integer=True)) 
        #     if cbar:
        #         cbar.remove()     
        #     cbar = plt.colorbar(sc, shrink=0.8, ax=ax4, pad=0.13)  
        #     cbar.set_label('Iteration')  
   
        sc = ax4.scatter(pose_error, energy, rns, s=50+n_iter,  c=n_iter, cmap=cm.plasma, alpha=0.5)
        # Update colorbar properties  
        ax4.set_title("Budget utilization")
        ax4.set_ylabel("Energy (Wh)")
        ax4.set_xlabel(r"Pose error ($E_{dist}$)")  
        ax4.set_zlabel(r"$\mathcal{T}_{cr}$")
        plt.tick_params(labelsize=8, direction="in")
        ax4.xaxis.set_major_locator(MaxNLocator(integer=True)) 
        if cbar:
            cbar.remove()     
        cbar = plt.colorbar(sc, shrink=0.8, ax=ax4, pad=0.13)  
        cbar.set_label('Iteration')                     
        
        plt.subplots_adjust(hspace=0.25, wspace=0.185, top=0.955, bottom=0.065, left=0.075, right=0.98)

if __name__ == "__main__":
    ani = animation.FuncAnimation(fig, live_result, interval=1000)
    plt.show()