/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelper(Node** node, uint depth, std::vector<float> point, int id)
	{
		if (*node == NULL)
			*node = new Node(point, id);
		else
		{
			//calculate current dimension (x, y or z)
			uint cd = depth % 3;
			if (point[cd] < (*node)->point[cd])
				insertHelper(&((*node)->left), depth+1, point, id);
			else
				insertHelper(&((*node)->right), depth+1, point, id);
		}
		
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		insertHelper(&root, 0, point, id);

	}

	void searchHelper(std::vector<float> target, Node* node, uint depth, float distanceTol, std::vector<int>& ids)
	{
		if(node != NULL)
		{
			// check if within search box
			float x = node->point[0];
			float y = node->point[1];
			float z = node->point[2];
			float xmin = target[0] - distanceTol;
			float xmax = target[0] + distanceTol;
			float ymin = target[1] - distanceTol;
			float ymax = target[1] + distanceTol;
			float zmin = target[2] - distanceTol;
			float zmax = target[2] + distanceTol;
			if(x >= xmin && x <= xmax && y >= ymin && y <= ymax && z >= zmin && z <= zmax)
			{
				// check if within tolerance radius
				float dist = sqrt((target[0]-x)*(target[0]-x) + (target[1]-y)*(target[1]-y) + (target[2]-z)*(target[2]-z));
				if(dist <= distanceTol)
					ids.push_back(node->id);
			}

			// check across boundary. if the box limits extend past left or right tree nodes
			// search there as well
			if((target[depth%3]-distanceTol) < (node->point[depth%3]))
				searchHelper(target, node->left, depth+1, distanceTol, ids);
			if((target[depth%3]+distanceTol) > (node->point[depth%3]))
				searchHelper(target, node->right, depth+1, distanceTol, ids);
		}

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}
	

};




