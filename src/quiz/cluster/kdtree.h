#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node {
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

void insert_recur(Node **n, std::vector<float> point, int id, int depth);

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
        search_recur(&root, &ids, &target, distanceTol, 0);
		return ids;
	}
};

void search_recur(Node **node, std::vector<int> &ids, const std::vector<float> target, const float distanceTol, int depth) {
    // check if current node is a candidate
    if (abs((*n)->point[0] - target[0])<distanceTol && abs((*n)->point[1] - target[1])<distanceTol) {
        // TODO: do some circle math
        ids->push_back((*n)->id);
    }

    // check if each child is a candidate
    int cd = depth % 2;
    if ((*n)->left.point[cd]


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

