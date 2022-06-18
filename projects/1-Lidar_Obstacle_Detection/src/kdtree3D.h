#include "render/render.h"

// Structure to represent node of kd tree
struct Node
{
	pcl::PointXYZI point;
	int id;
	Node *left;
	Node *right;

	Node(pcl::PointXYZI arr, int setId) : point(arr), id(setId), left(nullptr), right(nullptr) {}

	~Node()
	{
		if (left != nullptr)
		{
			delete left;
			left = nullptr;
		}
		if (right != nullptr)
		{
			delete right;
			right = nullptr;
		}
	}
};

struct KdTree
{
	Node *root;

	KdTree() : root(nullptr) {}

	~KdTree()
	{
		if (root != nullptr)
		{
			delete root;
			root = nullptr;
		}
	}

	void insertHelper(Node **node, int depth, pcl::PointXYZI point, int id)
	{
		// Tree is empty
		if (*node == nullptr)
		{
			*node = new Node{point, id};
		}
		else
		{
			// calculate current din
			int currentDimension = depth % 3; // 3D point
			if (currentDimension == 0)
			{
				if (point.x < (*node)->point.x)
				{
					insertHelper(&((*node)->left), depth + 1, point, id);
				}
				else
				{
					insertHelper(&((*node)->right), depth + 1, point, id);
				}
			}
			else if (currentDimension == 1)
			{
				if (point.y < (*node)->point.y)
				{
					insertHelper(&((*node)->left), depth + 1, point, id);
				}
				else
				{
					insertHelper(&((*node)->right), depth + 1, point, id);
				}
			}
			else
			{
				if (point.z < (*node)->point.z)
				{
					insertHelper(&((*node)->left), depth + 1, point, id);
				}
				else
				{
					insertHelper(&((*node)->right), depth + 1, point, id);
				}
			}
		}
	}

	void insert(pcl::PointXYZI point, int id)
	{
		// the function should create a new node and place correctly with in the root
		insertHelper(&root, 0, point, id);
	}

	void searchHelper(pcl::PointXYZI target, Node *node, int depth, float distanceTol, std::vector<int> &ids)
	{
		if (node != NULL)
		{
			const float deltaX = node->point.x - target.x;
			const float deltaY = node->point.y - target.y;
			const float deltaZ = node->point.z - target.z;

			if ((deltaX >= -distanceTol && deltaX <= distanceTol) &&
				(deltaY >= -distanceTol && deltaY <= distanceTol) &&
				(deltaZ >= -distanceTol && deltaZ <= distanceTol))
			{
				const float distance = sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);
				if (distance <= distanceTol)
				{
					ids.push_back(node->id);
				}
			}
			// check across boundary
			if (depth % 3 == 0) // 3D points
			{
				if (deltaX > -distanceTol)
				{
					searchHelper(target, node->left, depth + 1, distanceTol, ids);
				}
				if (deltaX < distanceTol)
				{
					searchHelper(target, node->right, depth + 1, distanceTol, ids);
				}
			}
			else if (depth % 3 == 1)
			{
				if (deltaY > -distanceTol)
				{
					searchHelper(target, node->left, depth + 1, distanceTol, ids);
				}
				if (deltaY < distanceTol)
				{
					searchHelper(target, node->right, depth + 1, distanceTol, ids);
				}
			}
			else
			{
				if (deltaZ > -distanceTol)
				{
					searchHelper(target, node->left, depth + 1, distanceTol, ids);
				}
				if (deltaZ < distanceTol)
				{
					searchHelper(target, node->right, depth + 1, distanceTol, ids);
				}
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(pcl::PointXYZI target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}
};
