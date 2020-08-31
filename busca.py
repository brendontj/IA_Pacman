# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
#
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In busca.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""
from lib.searchProblem import SearchProblem

import lib.util as util
from lib.util import Stack, Queue, PriorityQueue
'''
Uso da pilha:
stack = Stack()
stack.push(element)
l = stack.list # Lista de elementos na pilha
e = stack.pop()

Uso da fila:
queue = Queue()
queue.push(element)
l = queue.list # Lista de elementos na fila
e = queue.pop()

Uso da fila de prioridades:
queuep = PriorityQueue()
queuep.push(element, value) # Valores baixos indicam maior prioridade
queuep.update(element, value) # Atualiza o valor de prioridade  de um elemento
l = queuep.heap # Lista de elementos na fila
e = queuep.pop()
'''

def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Busca primeiro os nos mais profundos na arvore de busca

    Seu algoritmo deve retornar uma lista de acoes que atinja o objetivo.
    """

    if problem.isGoalState(problem.getStartState()):
        return []

    stack = Stack()
    visited = []
    path = [] 

    stack.push((problem.getStartState(),path))

    while not stack.isEmpty():
        current_state, path = stack.pop()
        if problem.isGoalState(current_state):
            return path

        visited.append(current_state)

        for successor in problem.getSuccessors(current_state):
            if successor[0] not in visited:
                newPath = path + [successor[1]]
                stack.push((successor[0], newPath))

def breadthFirstSearch(problem):
    """Busca primeiro os nos menos profundos na arvore de busca"""
    if problem.isGoalState(problem.getStartState()):
        return []

    queue = Queue()
    visited = []
    path = [] 

    queue.push((problem.getStartState(),path))

    while not queue.isEmpty():
        current_state, path = queue.pop()
        if problem.isGoalState(current_state):
            return path

        visited.append(current_state)
        for successor in problem.getSuccessors(current_state):
            if successor[0] not in visited and successor[0] not in (state[0] for state in queue.list):
                newPath = path + [successor[1]]
                queue.push((successor[0], newPath))

def uniformCostSearch(problem):
    """Busca primeiro os nos com o menor custo total"""

    if problem.isGoalState(problem.getStartState()):
        return []

    queue = PriorityQueue()
    visited = [] 
    path = [] 

    queue.push((problem.getStartState(),[]),0)

    while not queue.isEmpty():

        current_state,path = queue.pop()
        if problem.isGoalState(current_state):
            return path

        visited.append(current_state)

        for successor in problem.getSuccessors(current_state):
            if successor[0] not in visited:
                if (successor[0] not in (state[2][0] for state in queue.heap)):
                    newPath = path + [successor[1]]
                    cost = problem.getCostOfActions(newPath)
                    queue.push((successor[0],newPath),cost)

                elif (successor[0] in (state[2][0] for state in queue.heap)):
                    for state in queue.heap:
                        if state[2][0] == successor[0]:
                            old_cost = problem.getCostOfActions(state[2][1])

                    new_cost = problem.getCostOfActions(path + [successor[1]])
                    if old_cost > new_cost:
                        newPath = path + [successor[1]]
                        queue.update((successor[0],newPath),new_cost)

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Busca primeiro os nos que tem a menor combinacao de custo total e heuristica"""

    if problem.isGoalState(problem.getStartState()):
        return []

    queue = PriorityQueue()
    path = []
    visited = []

    queue.push((problem.getStartState(), path, 0), 0)

    while not queue.isEmpty():
        current_state, path, cost = queue.pop()

        if problem.isGoalState(current_state):
            return path
        
        if current_state not in visited:
            visited.append(current_state)

            for next_state, action, cost_successor in problem.getSuccessors(current_state):
                newPath = path + [action]
                newCost = cost + cost_successor
                heuristicCost = newCost + heuristic(next_state,problem)
                queue.push((next_state, newPath, newCost),heuristicCost)
    
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
