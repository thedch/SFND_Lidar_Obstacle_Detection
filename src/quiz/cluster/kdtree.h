#include "../../render/render.h"
#include <cmath>


// Structure to represent node of kd tree
struct Node {
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId) : point(arr), id(setId), left(NULL), right(NULL) {}
};

void insert_recur(Node **n, std::vector<float> point, int id, int depth);
void search_recur(Node **n, std::vector<int> &ids, const std::vector<float> target,
	              const float distanceTol, int depth);


struct KdTree {
	Node* root;

	KdTree() : root(NULL) {}

	void insert(std::vector<float> point, int id) {
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
        insert_recur(&root, point, id, 0);
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
        search_recur(&root, ids, target, distanceTol, 0);
		return ids;
	}
};


/*
Given a target and box like this:
   [#####]
   [#####]
   [##T##]
   [#####]
   [#####]
<-#######]    (R_explore)
   [######->  (L_explore)

If point lies within the R_explore band, then explore right subtree, and same
for left.

In pseudo code:
if pt < (t+d): explore R
if pt > (t-d): explore L
*/
void search_recur(Node **n, std::vector<int> &ids, const std::vector<float> target,
	              const float distanceTol, int depth) {
	if (*n == NULL) {
		return;
	}

	float t_x = target[0];
	float t_y = target[1];
	float my_x = (*n)->point[0];
	float my_y = (*n)->point[1];

	if (abs(my_x - t_x) < distanceTol &&
		abs(my_y - t_y) < distanceTol) { // point within box!
		if (pow(my_x-t_x, 2) + pow(my_y-t_y, 2) < pow(distanceTol, 2)) {
			ids.push_back((*n)->id);
		}
	}

	if ((*n)->point[depth % 2] < target[depth % 2] + distanceTol) {
		search_recur(&((*n)->right), ids, target, distanceTol, depth+1);
	}
	if ((*n)->point[depth % 2] > target[depth % 2] - distanceTol) {
		search_recur(&((*n)->left), ids, target, distanceTol, depth+1);
	}
}


void insert_recur(Node **n, std::vector<float> point, int id, int depth) {
    if (*n == NULL) {
        *n = new Node(point, id);
        return;
    }

    float point_data = 0;
    float node_data = 0;
    if (depth % 2 == 0) { // even
        point_data = point[0];
        node_data = (*n)->point[0];
    } else { // odd
        point_data = point[1];
        node_data = (*n)->point[1];
    }

    if (point_data < node_data) {
        insert_recur(&((*n)->left), point, id, depth+1);
    } else {
        insert_recur(&((*n)->right), point, id, depth+1);
    }
}
