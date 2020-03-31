#include "ActiveList.h"

ActiveList::ActiveList()
{
}
ActiveList::~ActiveList()
{
}
bool cmp(ALNode& a, ALNode& b)
{
	return a.cost > b.cost;
}

ALNode ActiveList::getMin()
{
	ALNode node_temp;
	set<ALNode>::iterator iter_temp;
	iter_temp = Set.begin();
	node_temp.row = iter_temp->row;
	node_temp.col = iter_temp->col;
	node_temp.cost = iter_temp->cost;
	Set.erase(iter_temp);
	return node_temp;
}

void ActiveList::Insert(ALNode node)
{
	Set.insert(node);
}
void ActiveList::Clear()
{
	Set.clear();
}
void ActiveList::Update(ALNode node_a, ALNode node_b)
{
	bool flag = true;
	set<ALNode>::iterator iter;
	iter = Set.find(node_a);
	Set.erase(iter);
	Set.insert(node_b);
}