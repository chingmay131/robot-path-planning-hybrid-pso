#! /usr/bin/env python
# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import os
import ast
import numpy as np
from mpl_toolkits.mplot3d import Axes3D # <--- This is important for 3d plotting
import matplotlib.lines as mlines
from matplotlib.ticker import (AutoMinorLocator, MultipleLocator)
import glob

full_path = os.path.realpath(__file__)
path = os.path.dirname(full_path)
no_of_solutions = 20

def transform_obstacles(obstacles):
    transformed_obstacles = []
    scale = 0.5
        
    for artikel in obstacles:
        center_x = round(artikel[0] * 2) / 2
        center_y = round(artikel[1] * 2) / 2
        x1 = center_x - scale
        y1 = center_y - scale
        transformed_obstacles.append((x1, y1))
        
    return transformed_obstacles

class BestPathVisualization:
    def __init__(self, world_name):
        self.world_name = world_name
        self.create_path_viz()
    
    def draw_obstacles(self,ax):
        f = open(path+"/obstacles_"+self.world_name+".txt", "r")
        obstacles = f.readline()
        f.close()
        obstacles = ast.literal_eval(obstacles)
        transformed_obstacles = transform_obstacles(obstacles)
        
        for artikel in transformed_obstacles:
            x = artikel[1]
            y = artikel[0]
            rect = patches.Rectangle((x,y),1,1,linewidth=1,facecolor='burlywood')
            ax.add_patch(rect)
    
    def draw_paths(self,ax):
        colours = ["red","green","blue"]
        target_directory = path+'/'+self.world_name
        filename = 'path_'+self.world_name
        list_of_files = glob.glob(target_directory+'/'+filename+'*.txt')
        latest_file = max(list_of_files, key=os.path.getctime)
        
        paths = []
        f = open(latest_file, "r")
        for x in f:
            paths.append(ast.literal_eval(x))
            
        for idx, algo in enumerate(paths):
            width = 2.0
            aa = algo["algorithm"]
            ap = algo["path"]
            ab = algo["best_among_all"]
            
            if ab:
              lbl = aa+" (optimum)"
            else:
              lbl = aa
            
            xs, ys = zip(*ap)
            ax.plot(xs, ys, '.-', lw=width, color=colours[idx], ms=10, label=lbl)
    
    def create_path_viz(self):
        fig, ax = plt.subplots(figsize=(5, 5))
        ax.plot(-4.5, -4.5, 'yd')
        ax.plot(4.5, 4.5, 'y*')
        self.draw_obstacles(ax)
        self.draw_paths(ax)
        
        plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc='lower left', \
         ncol=2, mode="expand", borderaxespad=0.)
           
        ax.set_xlim(-5, 5)
        ax.set_ylim(5, -5)
        # Change major ticks to show every 1.
        ax.xaxis.set_major_locator(MultipleLocator(1))
        ax.yaxis.set_major_locator(MultipleLocator(1))        
        # Change minor ticks to show every 0.5. (1/2 = 0.5)
        ax.xaxis.set_minor_locator(AutoMinorLocator(2))
        ax.yaxis.set_minor_locator(AutoMinorLocator(2))        
        # Turn grid on for both major and minor ticks and style minor slightly differently.
        ax.grid(which='major', color='#CCCCCC', linestyle='-')
        ax.grid(which='minor', color='#CCCCCC', linestyle=':')
        
        plt.show()#
        fig.savefig('figures/BestPath_'+self.world_name.capitalize(), bbox_inches='tight')

class SolutionVisualization:
    def __init__(self, world_name, algo_name):
        self.world_name = world_name
        self.algo_name = algo_name
        self.best_run = None
        self.best_length = None
        self.best_smoothness = None
        self.best_safety = None
        self.nparr = None
        self.pareto_frontier = None
        self.run_no = None
        self.create_solution_viz_3d()
        self.create_solution_viz_2d()
    
    def draw_solutions_3d(self,ax):
        directory = path+'/'+self.world_name+'_selected/'
        filename = self.algo_name+'_'+self.world_name
            
        list_of_files = glob.glob(directory+'/'+filename+'*.txt')
        latest_file = max(list_of_files, key=os.path.getctime)

        with open(latest_file) as myfile:
            nparr = [ast.literal_eval(next(myfile))[0] for x in range(no_of_solutions)]
            nparr = np.array(nparr)
        
        with open(latest_file, 'r') as f:
            lines = f.read().splitlines()
            pareto_frontier = ast.literal_eval(lines[-3])
            best_solution = ast.literal_eval(lines[-1])
            run_no = int(best_solution['run_no'])
        self.nparr = nparr
        self.pareto_frontier = pareto_frontier
        self.run_no = run_no
            
        for i in range(len(nparr)):
            if i == run_no:
                self.best_run = str(run_no+1)
                self.best_length = str(nparr[i,0])
                self.best_smoothness = str(nparr[i,1])
                self.best_safety = str(nparr[i,2])
                ax.scatter(nparr[i,0], nparr[i,1], nparr[i,2], c='r')
            elif i in pareto_frontier:
                ax.scatter(nparr[i,0], nparr[i,1], nparr[i,2], c='y')
            else:
                ax.scatter(nparr[i,0], nparr[i,1], nparr[i,2], c='g')
                
            ax.text(nparr[i,0], nparr[i,1], nparr[i,2], '%s' % (str(i+1)), size=10, \
             zorder=1, color='k')
    
    def draw_legend(self,ax):
        scatter1_proxy = mlines.Line2D([0],[0], linestyle="none")
        scatter2_proxy = mlines.Line2D([0],[0], linestyle="none")
        scatter3_proxy = mlines.Line2D([0],[0], linestyle="none", c='r', marker = 'o')
        scatter4_proxy = mlines.Line2D([0],[0], linestyle="none")
        scatter5_proxy = mlines.Line2D([0],[0], linestyle="none")
        scatter6_proxy = mlines.Line2D([0],[0], linestyle="none")
        scatter7_proxy = mlines.Line2D([0],[0], linestyle="none")
        scatter8_proxy = mlines.Line2D([0],[0], linestyle="none", c='y', marker = 'o')
        scatter9_proxy = mlines.Line2D([0],[0], linestyle="none", c='g', marker = 'o')
        
        ax.legend([scatter1_proxy, scatter2_proxy, scatter3_proxy, scatter4_proxy, \
         scatter5_proxy, scatter6_proxy, scatter7_proxy, scatter8_proxy, scatter9_proxy], \
         [self.algo_name, 'World: '+self.world_name.capitalize(), 'Best solution', \
         '- Run: '+self.best_run, '- Length: '+self.best_length, \
         '- Smoothness: '+self.best_smoothness, '- Safety: '+self.best_safety, \
         'Pareto frontier', 'Other solutions'], numpoints = 1, bbox_to_anchor=(1.05, 1), \
         loc='upper left')
    
    def create_solution_viz_3d(self):
        fig = plt.figure(figsize=(12.8,9.6))
        ax = fig.add_subplot(111, projection='3d')
        
        self.draw_solutions_3d(ax)
        self.draw_legend(ax)
        
        ax.set_xlabel('Length')
        ax.set_ylabel('Smoothness')
        ax.set_zlabel('Safety')

        plt.show()
        fig.savefig('figures/solutions_3d/BestSolution_'+self.algo_name+'_'+self.world_name.capitalize()+'_3D', \
         bbox_inches='tight')
    
    def create_solution_viz_2d(self):
        fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(20,5))
        nparr = self.nparr
        pareto_frontier = self.pareto_frontier
        run_no = self.run_no
            
        for i in range(len(nparr)):
            if i == run_no:
                ax1.scatter(nparr[i,0], nparr[i,1], c='r')
                ax2.scatter(nparr[i,1], nparr[i,2], c='r')
                ax3.scatter(nparr[i,0], nparr[i,2], c='r')
            elif i in pareto_frontier:
                ax1.scatter(nparr[i,0], nparr[i,1], c='y')
                ax2.scatter(nparr[i,1], nparr[i,2], c='y')
                ax3.scatter(nparr[i,0], nparr[i,2], c='y')
            else:
                ax1.scatter(nparr[i,0], nparr[i,1], c='g')
                ax2.scatter(nparr[i,1], nparr[i,2], c='g')
                ax3.scatter(nparr[i,0], nparr[i,2], c='g')
                
            ax1.text(nparr[i,0], nparr[i,1], '%s' % (str(i+1)), size=10, zorder=1, color='k')
            ax2.text(nparr[i,1], nparr[i,2], '%s' % (str(i+1)), size=10, zorder=1, color='k')
            ax3.text(nparr[i,0], nparr[i,2], '%s' % (str(i+1)), size=10, zorder=1, color='k')
        
        plt.subplots_adjust(wspace=0.25)
        ax1.set_xlabel('Length')
        ax1.set_ylabel('Smoothness')
        ax2.set_xlabel('Smoothness')
        ax2.set_ylabel('Safety')
        ax3.set_xlabel('Length')
        ax3.set_ylabel('Safety')

        plt.show()
        fig.savefig('figures/solutions_2d/BestSolution_'+self.algo_name+'_'+self.world_name.capitalize()+'_2D', \
         bbox_inches='tight')

class AllPathsVisualization:
    def __init__(self, world_name, algo_name):
        self.world_name = world_name
        self.algo_name = algo_name
        self.create_path_viz()
    
    def draw_obstacles(self,ax):
        f = open(path+"/obstacles_"+self.world_name+".txt", "r")
        obstacles = f.readline()
        f.close()
        obstacles = ast.literal_eval(obstacles)
        transformed_obstacles = transform_obstacles(obstacles)
        
        for artikel in transformed_obstacles:
            x = artikel[1]
            y = artikel[0]
            rect = patches.Rectangle((x,y),1,1,linewidth=1,facecolor='burlywood')
            ax.add_patch(rect)
    
    def draw_paths(self,ax):
        directory = path+'/'+self.world_name+'_selected/'
        filename = self.algo_name+'_'+self.world_name
            
        list_of_files = glob.glob(directory+'/'+filename+'*.txt')
        latest_file = max(list_of_files, key=os.path.getctime)

        with open(latest_file) as myfile:
            nparr = [ast.literal_eval(next(myfile))[1] for x in range(no_of_solutions)]
        
        with open(latest_file, 'r') as f:
            lines = f.read().splitlines()
            best_solution = ast.literal_eval(lines[-1])
            run_no = int(best_solution['run_no'])
            
        for i in range(len(nparr)):
            xs, ys = zip(*nparr[i])
            if i == run_no:
                brxs = xs
                brys = ys
            else:
                ax.plot(xs, ys, '-', c='g', ms=10)

        ax.plot(brxs, brys, '.-', lw=2.0, c='r', ms=10, \
         label='Best path in '+self.world_name.capitalize()\
          +' level world \nusing '+self.algo_name)
    
    def create_path_viz(self):
        fig, ax = plt.subplots(figsize=(5, 5))
        ax.plot(-4.5, -4.5, 'yd')
        ax.plot(4.5, 4.5, 'y*')
        self.draw_obstacles(ax)
        self.draw_paths(ax)
        
        plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc='lower left', \
         ncol=2, mode="expand", borderaxespad=0.)
           
        ax.set_xlim(-5, 5)
        ax.set_ylim(5, -5)
        # Change major ticks to show every 1.
        ax.xaxis.set_major_locator(MultipleLocator(1))
        ax.yaxis.set_major_locator(MultipleLocator(1))        
        # Change minor ticks to show every 0.5. (1/2 = 0.5)
        ax.xaxis.set_minor_locator(AutoMinorLocator(2))
        ax.yaxis.set_minor_locator(AutoMinorLocator(2))        
        # Turn grid on for both major and minor ticks and style minor slightly differently.
        ax.grid(which='major', color='#CCCCCC', linestyle='-')
        ax.grid(which='minor', color='#CCCCCC', linestyle=':')
        
        plt.show()
        fig.savefig('figures/paths/AllPaths_'+self.algo_name+'_'+self.world_name.capitalize(), \
         bbox_inches='tight')

BestPathVisualization('easy')
BestPathVisualization('medium')
BestPathVisualization('hard')

SolutionVisualization('easy','PSOFS')
SolutionVisualization('medium','PSOFS')
SolutionVisualization('hard','PSOFS')
SolutionVisualization('easy','PSOA')
SolutionVisualization('medium','PSOA')
SolutionVisualization('hard','PSOA')
SolutionVisualization('easy','PSOD')
SolutionVisualization('medium','PSOD')
SolutionVisualization('hard','PSOD')

AllPathsVisualization('easy','PSOFS')
AllPathsVisualization('medium','PSOFS')
AllPathsVisualization('hard','PSOFS')
AllPathsVisualization('easy','PSOA')
AllPathsVisualization('medium','PSOA')
AllPathsVisualization('hard','PSOA')
AllPathsVisualization('easy','PSOD')
AllPathsVisualization('medium','PSOD')
AllPathsVisualization('hard','PSOD')