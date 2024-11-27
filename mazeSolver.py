import numpy as np
import heapq
import matplotlib.pyplot as plt
from collections import deque

class MazeSolver:
    def __init__(self, maze, start, end):
        """
        Initialise la classe MazeSolver avec le labyrinthe, les positions de départ et d'arrivée.
        Vérifie que les positions de départ et d'arrivée sont valides au moment de l'initialisation.
        """
        self.maze = maze  # Le labyrinthe représenté par une matrice
        self.start = start  # Position de départ
        self.end = end  # Position d'arrivée
        self.check_positions()  # Vérifie que les positions de départ et d'arrivée sont valides

    def check_positions(self):
        """
        Vérifie si les positions de départ et d'arrivée sont valides (elles ne doivent pas être sur un mur).
        Si l'une des positions est invalide, une erreur est levée.
        """
        if self.maze[self.start[0], self.start[1]] == 1:
            raise ValueError("La position de départ est sur un mur. Choisissez une cellule de chemin valide.")
        if self.maze[self.end[0], self.end[1]] == 1:
            raise ValueError("La position d'arrivée est sur un mur. Choisissez une cellule de chemin valide.")

    def display_maze(self, resolution=16):
        """
        Affiche le labyrinthe en marquant les positions de départ et d'arrivée.
        """
        resolution = resolution or self.maze.shape[0]  # Utilise la taille du labyrinthe si la résolution n'est pas spécifiée
        # Affiche le labyrinthe avec des lignes de grille
        fig, ax = plt.subplots(figsize=(8, 8))
        ax.imshow(self.maze, cmap='binary')
        
        # Marque les positions de départ et d'arrivée
        ax.scatter(self.start[1], self.start[0], color='blue', s=100, label='Départ (O)')
        ax.scatter(self.end[1], self.end[0], color='red', s=100, label='Arrivée (X)')
        
        # Calcule l'espacement des lignes de grille et applique un décalage de 0.5
        x_step = self.maze.shape[1] / resolution
        y_step = self.maze.shape[0] / resolution
        
        # Trace les lignes verticales de la grille avec un décalage de 0.5
        for i in range(1, resolution):
            ax.axvline(i * x_step - 0.5, color='blue', linestyle='-', linewidth=0.5, alpha=0.3, zorder=0)
        
        # Trace les lignes horizontales de la grille avec un décalage de 0.5
        for i in range(1, resolution):
            ax.axhline(i * y_step - 0.5, color='blue', linestyle='-', linewidth=0.5, alpha=0.3, zorder=0)
        
        # Inverse l'axe y et ajoute un titre
        plt.title(f'Labyrinthe avec une grille de {resolution}x{resolution}')
        plt.show()

    @staticmethod
    def heuristic(a, b):
        """
        Fonction heuristique pour l'algorithme A*, qui calcule la distance de Manhattan entre deux points.
        """
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def get_neighbors(self, position):
        """
        Retourne tous les voisins valides d'une position donnée, y compris les déplacements diagonaux.
        """
        x, y = position
        neighbors = []
        
        # Vérifie tous les huit déplacements possibles, y compris les diagonales
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.maze.shape[0] and 0 <= ny < self.maze.shape[1] and self.maze[nx, ny] == 0:
                # Attribue un coût de 1 pour les déplacements verticaux/horizontaux et 1.4 pour les déplacements diagonaux
                cost = 1.4 if abs(dx) + abs(dy) == 2 else 1
                neighbors.append((nx, ny, cost))
                
        return neighbors

    @staticmethod
    def reconstruct_path(came_from, current):
        """
        Reconstruit le chemin depuis l'objectif jusqu'au départ à l'aide du dictionnaire came_from.
        """
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def astar(self):
        """
        Algorithme A* pour trouver le chemin le plus court du départ à l'arrivée.
        Retourne le chemin s'il est trouvé, sinon retourne une liste vide.
        """
        open_set = []
        heapq.heappush(open_set, (0, self.start))  # Initialise avec le nœud de départ
        came_from = {}
        g_score = {self.start: 0}
        f_score = {self.start: self.heuristic(self.start, self.end)}
        
        while open_set:
            _, current = heapq.heappop(open_set)
            
            # Vérifie si nous avons atteint l'objectif
            if current == self.end:
                return self.reconstruct_path(came_from, current)
            
            # Explore les voisins
            for neighbor in self.get_neighbors(current):
                neighbor_position = neighbor[:2]  # Récupère la position (nx, ny)
                tentative_g_score = g_score[current] + neighbor[2]  # Utilise le coût du déplacement
                
                # Met à jour uniquement si ce chemin vers le voisin est meilleur
                if neighbor_position not in g_score or tentative_g_score < g_score[neighbor_position]:
                    came_from[neighbor_position] = current
                    g_score[neighbor_position] = tentative_g_score
                    f_score[neighbor_position] = tentative_g_score + self.heuristic(neighbor_position, self.end)
                    heapq.heappush(open_set, (f_score[neighbor_position], neighbor_position))
        
        return []  # Si aucun chemin n'est trouvé, retourne un chemin vide


    def bfs(self):
        """
        Algorithme BFS (Breadth-First Search) pour trouver le chemin le plus court du point de départ
        au point d'arrivée dans le labyrinthe.
        
        Retourne le chemin si trouvé, sinon une liste vide.
        """
        # La queue pour gérer les positions à explorer
        queue = deque([self.start])
        # Dictionnaire pour tracer d'où chaque position vient
        came_from = {self.start: None}
        
        # Tant qu'il y a des positions à explorer
        while queue:
            current = queue.popleft()  # Récupérer la position en haut de la queue
            
            # Si on a atteint le but, on reconstruit le chemin
            if current == self.end:
                return self.reconstruct_path(came_from, current)
            
            # Explorer les voisins valides
            for neighbor in self.get_neighbors(current):
                neighbor_position = neighbor[:2]
                if neighbor_position not in came_from:
                    # Marquer cette position comme visitée et ajouter à la queue
                    queue.append(neighbor_position)
                    came_from[neighbor_position] = current
        
        return []  # Si aucun chemin n'est trouvé, retourner une liste vide

    

    def get_path(self,algo):
        if algo == 'astar':
            path = self.astar()  # Si A*, on appelle la méthode pour A*
        elif algo == 'bfs':
            path = self.bfs()[1:]  # Si BFS, on appelle la méthode pour BFS
        else:
            print("Algorithme inconnu. Veuillez choisir 'astar' ou 'bfs'.")
            path = []  # Si un algorithme invalide est passé, on ne trace pas de chemin
            
        return path
    
    def display_path(self, resolution=16, linewidth=3, algo='astar'):
        """
        Affiche le labyrinthe avec le départ, l'arrivée et le chemin calculé.
        
        Paramètres :
        - resolution (int) : Contrôle la résolution de la grille en fonction de la hauteur ou de la largeur du labyrinthe.
        - linewidth (float) : L'épaisseur des lignes de la grille.
        - algo (str) : L'algorithme utilisé pour calculer le chemin ('astar' ou 'bfs').
        """
        # Définit la résolution en fonction de la hauteur ou de la largeur du labyrinthe
        resolution = resolution or self.maze.shape[0]  # Résolution par défaut = hauteur du labyrinthe
        
        # Créer une nouvelle figure pour l'affichage
        fig, ax = plt.subplots(figsize=(8, 8))
        ax.imshow(self.maze, cmap='binary')

        # Trace les points de départ et d'arrivée
        ax.scatter(self.start[1], self.start[0], color='blue', s=100, label='Départ (O)', edgecolor='black')
        ax.scatter(self.end[1], self.end[0], color='red', s=100, label='Arrivée (X)', edgecolor='black')

        # Calcule l'espacement des lignes de la grille
        x_step = self.maze.shape[1] / resolution
        y_step = self.maze.shape[0] / resolution
        
        # Trace les lignes verticales et horizontales de la grille avec l'épaisseur donnée
        for i in range(1, resolution):
            ax.axvline(i * x_step - 0.5, color='blue', linestyle='-', linewidth=0.5, alpha=0.2, zorder=0)
            ax.axhline(i * y_step - 0.5, color='blue', linestyle='-', linewidth=0.5, alpha=0.2, zorder=0)

        # Choisir l'algorithme de calcul du chemin (A* ou BFS)
        path = self.get_path(algo=algo)

        # Si un chemin a été trouvé, le dessiner
        if path:
            path_x, path_y = zip(*path)
            ax.plot(path_y, path_x, color='green', linewidth=linewidth, zorder=10, label='Chemin')
        else:
            print("Aucun chemin trouvé.")

        # Affichage
        plt.title('Labyrinthe avec le chemin ({})'.format(algo))
        ax.legend()
        plt.show()

