#include <algorithm>
#include <set>

using namespace std;

class ALNode
{
public:
	int row;
	int col;
	double cost;
public:
	ALNode() {};
	ALNode(int Row, int Col, double Cost) {
		row = Row;
		col = Col;
		cost = Cost;
	}
	bool operator < (const ALNode& b) const
	{
		return cost < b.cost;
	}
	bool operator == (const ALNode& b) const
	{
		if (row == b.row || col == b.col || cost == b.cost)return true;
		return false;
	}
};
class ActiveList
{
private:
	set<ALNode> Set;
public:
	ActiveList();
	~ActiveList();
	ALNode getMin();	// ���ز���AL����СcostԪ��
	void Insert(ALNode node);
	void Clear();
	void Update(ALNode node_a, ALNode node_b);	// �ҵ��ڵ�node_a�����³�node_b
	bool IS_EMPTY()
	{
		return Set.empty();
	}
};

