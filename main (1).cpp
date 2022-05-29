#include <cmath>
#include <iostream>
#include <vector>
using namespace std;

class Node {
public:
  int matrix[3][3]; // Data
  int g, h;         // Cost

  int cost() { return g + h; }

  Node() {}
  Node(int matrix[3][3]) {
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        this->matrix[i][j] = matrix[i][j];
      }
    }
    this->g = 0;
    this->h = 0;
  }

  void location(int val, int *col, int *row) {
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        if (this->matrix[i][j] == val) {
          *col = i;
          *row = j;
        }
      }
    }
  }

  void print() {
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        cout << this->matrix[i][j] << " ";
      }
      cout << endl;
    }
  }

  bool operator==(Node n) {
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        if (n.matrix[i][j] != this->matrix[i][j]) {
          return false;
        }
      }
    }
    return true;
  }
};

class Problem : public Node {
public:
  // passes in for search algorithm

  int choice;

  vector<Node> frontier;
  vector<Node> visitedTasks;
  Node goal;

  Problem(int start[3][3], int goal[3][3], int choice) {
    cout << "Start" << endl;
    Node temp1(goal);
    this->goal = temp1;
    cout << "Mid" << endl;
    Node temp2(start);
    this->frontier.push_back(temp2);
    this->choice = choice;
  }

  Node schedular() {
    if (this->frontier.empty())
      return NULL;

    Node max = frontier[0];
    for (int i = 1; i < frontier.size(); i++) {
      if (frontier[i].cost() < max.cost()) {
        max = frontier[i];
      }
    }
    for (int i = 0; i < frontier.size(); i++) {
      if (frontier[i] == max) {
        // frontier[i] = frontier[frontier.size() - 1];
        // frontier[frontier.size() - 1] = max;
        // cout << "Deleted: " << endl;
        // frontier[i].print();
        frontier.erase(frontier.begin() + i);
        break;
      }
    }
    // frontier.pop_back();
    visitedTasks.push_back(max);
    return max;
  }

  void generateCost(Node parent, Node *curr) {
    if (choice == 1) {
      costUCS(parent, curr);
    } else if (choice == 2) {
      costAMT(parent, curr);
    } else if (choice == 3) {
      costAED(parent, curr);
    } else {
      cout << "Invalid Input\n";
      exit(0);
    }
  }

  // Uniform Cost Search
  void costUCS(Node parent, Node *curr) {
    curr->g = parent.g + 1;
    curr->h = 0;
  }

  // A* Misplaced Tile Heuristic
  void costAMT(Node parent, Node *curr) {
    curr->g = parent.g + 1;

    curr->h = 0;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        if (curr->matrix[i][j] != goal.matrix[i][j]) {
          curr->h += 1;
        }
      }
    }
  }

  // A* Euclidean Distance Heuristic
  void costAED(Node parent, Node *curr) {
    curr->g = parent.g + 1;

    curr->h = 0;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        int goalI, goalJ;
        goal.location(curr->matrix[i][j], &goalI, &goalJ);
        curr->h =
            (int)sqrt((goalI - i) * (goalI - i) + (goalJ - j) * (goalJ - j));
      }
    }
  }

  bool checkRepeat(Node curr) {
    // pass in the current node and compare it to the nodes in the frontier and
    // visited
    for (Node n : frontier) {
      bool eq = true;
      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
          if (n.matrix[i][j] != curr.matrix[i][j]) {
            eq = false;
          }
        }
      }
      if (eq) {
        return true;
      }
    }
    // visited
    for (Node n : visitedTasks) {
      bool eq = true;
      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
          if (n.matrix[i][j] != curr.matrix[i][j]) {
            eq = false;
          }
        }
      }
      if (eq) {
        return true;
      }
    }

    return false;
  }

  void possibleMoves(Node curr) {
    // find the x and y of zero
    int zeroX, zeroY;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        if (curr.matrix[i][j] == 0) {
          zeroX = j;
          zeroY = i;
        }
      }
    }

    // Move 0 right
    if (zeroX != 2) {
      Node child(curr.matrix);
      int temp = child.matrix[zeroY][zeroX];
      child.matrix[zeroY][zeroX] = child.matrix[zeroY][zeroX + 1];
      child.matrix[zeroY][zeroX + 1] = temp;
      if (!checkRepeat(child)) {
        generateCost(curr, &child);
        this->frontier.push_back(child);
      }
      // Move 0 left
    }
    if (zeroX != 0) {
      Node child(curr.matrix);
      int temp = child.matrix[zeroY][zeroX];
      child.matrix[zeroY][zeroX] = child.matrix[zeroY][zeroX - 1];
      child.matrix[zeroY][zeroX - 1] = temp;
      if (!checkRepeat(child)) {
        generateCost(curr, &child);
        this->frontier.push_back(child);
      }
      // Move 0 up
    }
    if (zeroY != 2) {
      Node child(curr.matrix);
      int temp = child.matrix[zeroY][zeroX];
      child.matrix[zeroY][zeroX] = child.matrix[zeroY + 1][zeroX];
      child.matrix[zeroY + 1][zeroX] = temp;
      if (!checkRepeat(child)) {
        generateCost(curr, &child);
        this->frontier.push_back(child);
      }
      // Move 0 down
    }
    if (zeroY != 0) {
      Node child(curr.matrix);
      int temp = child.matrix[zeroY][zeroX];
      child.matrix[zeroY][zeroX] = child.matrix[zeroY - 1][zeroX];
      child.matrix[zeroY - 1][zeroX] = temp;
      if (!checkRepeat(child)) {
        generateCost(curr, &child);
        this->frontier.push_back(child);
      }
    }
  }

  bool isGoal(Node n) {
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        if (n.matrix[i][j] != goal.matrix[i][j]) {
          return false;
        }
      }
    }

    return true;
  }
};

int main() {
  int initial_state[3][3] = {{1, 2, 3}, {4, 8, 0}, {7, 6, 5}};

  int final_state[3][3] = {{1, 2, 3}, {4, 5, 6}, {7, 8, 0}};
  int choice;
  cout << "Welcome to 861163782 8-puzzle solver\n";
  cout << "Type '1' to use a default puzzle, or '2' to enter your own puzzle\n";
  cin >> choice;

  if (choice == 1) {
    // load initial puzzle

  } else if (choice == 2) {
    // load own create matrix
    cout << "Enter the element of your 3x3 matrix:\n";
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        cin >> initial_state[i][j];
      }
    }

    cout << "Initial state of 3x3 matrix: ";
    for (int i = 0; i < 3; i++) {
      cout << endl;
      for (int j = 0; j < 3; j++) {
        cout << initial_state[i][j] << " ";
      }
    }

  } else {
    cout << "Invalid Input\n";
    exit(0);
  }

  cout << endl;
  int algo_choice;
  cout << "Enter your choice of algorithm:\n";
  cout << "1: Uniform Cost Search\n";
  cout << "2: A* with the Misplaced Tile heuristic\n";
  cout << "3: A* with the Euclidean distance heuristic\n";
  cin >> algo_choice;

  if (algo_choice < 1 || algo_choice > 3) {
    cout << "Invalid Input\n";
    exit(0);
  }

  Problem probs(initial_state, final_state,
                algo_choice); // Create an object of Problem
  Node n(initial_state);
  int maxFrontierSize = 1;
  while (!probs.isGoal(n)) {
    if (probs.frontier.empty()) {
      // Quit
      cout << "No Moves" << endl;
      break;
    }
    n = probs.schedular();
    cout << "Current State: " << n.cost() << endl;
    n.print();
    cout << endl;

    probs.possibleMoves(n);
    if (probs.frontier.size() > maxFrontierSize)
      maxFrontierSize = probs.frontier.size();

    // cout << "Frontier" << endl;
    // if (!probs.frontier.empty()) {
    //	for(Node m : probs.frontier) m.print();
    //}
    // cin >> choice;
  }
  cout << "GOAL!!!" << endl;
  cout << endl;
  cout << "To solve this problem the search algorithm expanded a total of "
       << probs.visitedTasks.size() << " nodes." << endl;
  cout << "The maximum number of nodes in the queue at any one time: "
       << probs.frontier.size() << endl;
  cout << "The depth of the goal node was: " << n.g << endl;
}