#include<opencv2/opencv.hpp>
#include<iostream>
#include<opencv2/highgui/highgui_c.h>
#include<opencv2/imgproc.hpp>        
#include<vector>
#include<math.h>
#include<queue>
#include<climits>
#include<stack>
#define _math_defines_defined
#define _use_math_defines
#define pi 3.1415926
#define sqrt2  1.4142135623731 
using namespace std;
using namespace cv;
vector<Point> v;
vector<vector<vector<float> > > LCost;
Mat source;
Mat src;
Mat detected_edges;
Mat grey;
int safe = 0;
Point first;
Point current;
Mat xx;
//Mat res;
void initV() {
	v.push_back(Point(1, 0));
	v.push_back(Point(1, 1));
	v.push_back(Point(0, 1));
	v.push_back(Point(-1, 1));
	v.push_back(Point(-1, 0));
	v.push_back(Point(-1, -1));
	v.push_back(Point(0, -1));
	v.push_back(Point(1, -1));
}
const float Wz = 0.43f, Wg = 0.43f, Wd = 0.14f;
void initial_cost(int row, int col) {
	vector<vector<vector<float> > > temp(row, vector<vector<float> >(col, vector<float>(8)));
	swap(temp, LCost);
}
void initial_Fz()
{
	cvtColor(src, grey, COLOR_BGR2GRAY);
	bilateralFilter(grey, detected_edges, 15, 150, 3);
	Canny(detected_edges, detected_edges, 3, 9, 3);
	vector<vector<float> > Fz(src.rows, vector<float>(src.cols));
	for (int i = 0; i < src.rows; i++)
	{
		for (int j = 0; j < src.cols; j++) {
			if (detected_edges.at<uchar>(i, j) > 0)Fz[i][j] = 1.0f;
			else Fz[i][j] = 0.0f;
		}
	}
	for (int i = 0; i < src.rows; i++)
	{
		for (int j = 0; j < src.cols; j++)
		{
			//PixelNode& node = GetPN(i, j, Cols);
			for (int k = 0; k < 8; k++)
			{
				if (i + v[k].x<0 || i + v[k].x>src.rows || j + v[k].y<0 || j + v[k].y>src.cols)
					continue;
				LCost[i][j][k] += Wz * Fz[i][j];
			}
		}
	}
}


void initial_Fg_Fd()
{
	vector<vector<float> > Fg(src.rows, vector<float>(src.cols,0.0f));
	float minG = -1.0f;
	float maxG = 0.0f;
	Mat dx, dy;
	Scharr(grey, dx, src.depth(), 1, 0);
	Scharr(grey, dy, src.depth(), 0, 1);
	convertScaleAbs(dx, dx);
	convertScaleAbs(dy, dy);
	Mat dest;
	dest.create(dx.rows, dx.cols, CV_32FC1);
	for (int i = 0; i < src.rows; i++)
	{
		for (int j = 0; j < src.cols; j++)
		{
			float ix = dx.at<uchar>(i, j);
			float iy = dy.at<uchar>(i, j);
			Fg[i][j] = sqrt((ix) * (ix)+(iy) * (iy));
		}
	}
	for (int i = 0; i < src.rows; i++) {
		for (int j = 0; j < src.cols; j++) {
			for (int k = 0; k < 8; k++) {
				if (i + v[k].y < 0 || i + v[k].y >= src.rows || j + v[k].x < 0 || j + v[k].x >= src.cols)
					continue;
				maxG = max(Fg[i + v[k].y][j + v[k].x], maxG);
			}
			if (maxG == 0) dest.at<float>(i, j) = 0;
			else dest.at<float>(i, j) = 1 - Fg[i][j] / maxG;

		}
	}
	float x1, y1, x2, y2;
	float dp, dq;
	for (int i = 0; i < src.rows; i++)
	{
		for (int j = 0; j < src.cols; j++)
		{
			float sqt2;
			float max1 = 0;
			for (int k = 0; k < 8; k++)
			{
				float X, Y;
				X = v[k].x;
				Y = v[k].y;
				sqt2 = 1.0f;
				if (i + Y < 0 || i + Y >= src.rows || j + X < 0 || j + X >= src.cols)continue;
				if (Fg[i][j] != 0)
				{
					x1 = (float)dx.at<uchar>(i, j) / Fg[i][j];
					y1 = (float)dy.at<uchar>(i, j) / Fg[i][j];
					dp = (y1 *X- x1 *Y);
				}
				else dp = 0;
				max1 = max(max1, Fg[i + Y][j + X]);
				if (Fg[Y + i][X + j] != 0)
				{
					x2 = (float)dx.at<uchar>(i + Y, j + X) / Fg[i + Y][j + X];
					y2 = (float)dy.at<uchar>(i + Y, j + X) / Fg[i + Y][j + X];
					if (dp < 0) {dp = -dp;dq = x2 * Y - y2 * X;}
					else dq = y2 * X - x2 * Y;
				}
				else dq = 0;
				if (k % 2)dp/=sqrt2,dq /= sqrt2;
				LCost[i][j][k] = Wg * dest.at<float>(i, j) + Wd * 2.0/3.0*(acos(dp) + acos(dq)) / pi;
			}
		}
	}
}
void Initial() {
	initV();
	initial_cost(src.rows, src.cols);
	for (int i = 0; i < src.rows; i++)
		for (int j = 0; j < src.cols; j++)
			for (int k = 0; k < 8; k++)
				LCost[i][j][k] = 0;
	initial_Fz();
	initial_Fg_Fd();
}
struct node {
	int from_x, from_y;
	float cost = 2000;
	bool expand = false;
};
vector<vector<node> > Cost(2000, vector<node>(2000));
struct pq_node {
	int x, y;
	float cost;
};
struct cmp{
	bool operator()(pq_node a, pq_node b) { return a.cost > b.cost; }
};
void initial_cost_node() {
	cout << src.rows << " " << src.cols << endl;
	for (int i = 0; i < src.rows; i++) {
		for (int j = 0; j < src.cols; j++) {
			Cost[i][j].from_x = j;
			Cost[i][j].from_y = i;
			Cost[i][j].cost = 2000;
			Cost[i][j].expand = false;
		}
	}
}
void dij(int x, int y) {
	pq_node cur, seedpoint;
	int sum = 0;
	//对种子点进行设置
	initial_cost_node();
	seedpoint.x = x;
	seedpoint.y = y;
	seedpoint.cost = 0;
	priority_queue<pq_node, vector<pq_node>, cmp> pq;
	Cost[y][x].from_y = y;
	Cost[y][x].from_x = x;
	Cost[y][x].cost = 0;
	Cost[y][x].expand = true;
	pq.push(seedpoint);
	for (int i = 0; i < src.cols; i++) 
	{
		Cost[0][i].expand = true;
		Cost[src.rows - 1][i].expand = true;
	}
	for (int i = 0; i < src.rows; i++) 
	{
		Cost[i][0].expand = true;
		Cost[i][src.cols - 1].expand = true;
	}
	while (!pq.empty()) {
		cur = pq.top();
		pq.pop();
		sum++;
		if (abs(cur.x - x) > 300 || abs(cur.y - y) > 300)continue;
		Cost[cur.x][cur.y].expand = true;
		for (int i = 0; i < 8; i++) {
			int X, Y;
			X = v[i].x+ cur.x;
			Y = v[i].y+ cur.y;
			if ( X < 0 || X >= src.cols || Y < 0 || Y >= src.rows)continue;
			float upd=Cost[cur.y][cur.x].cost + LCost[cur.y][cur.x][i];
			if (Cost[Y][X].cost>upd && !Cost[Y][X].expand ) {
				Cost[Y][X].cost = upd;
				Cost[Y][X].from_x = cur.x;
				Cost[Y][X].from_y = cur.y;
				pq_node node;
				node.x =X;
				node.y =Y;
				node.cost = Cost[Y][X].cost;
				if (abs(node.x - x) > 300 || abs(node.y - y) > 300)continue;
				pq.push(node);
			}
		}
	}
}
void line(int x, int y) {
	Mat temp;
	source.copyTo(temp);
	int ox, oy;
	ox = Cost[y][x].from_x;
	oy = Cost[y][x].from_y;
	while (ox != x || oy != y) {
		x = ox;
		y = oy;
		ox = Cost[y][x].from_x;
		oy = Cost[y][x].from_y;
		circle(temp, Point(ox, oy), 1, Scalar(0, 255, 0), -1);
	}
	imshow("win_name", temp);
}
vector<Point> pts;
void mark(int x, int y) {
		current = Point(x, y);
		circle(src, Point(x, y), 2, Scalar(255, 0, 0), -1,8,0);
		stack<Point>s;
		s.push(Point(x, y));
		int ox, oy;
		ox = Cost[y][x].from_x;
		oy = Cost[y][x].from_y;
		cout << ox << "  " << oy << endl;
		
		while (ox != x || oy != y) {
			x = ox;
			y = oy;
			ox = Cost[y][x].from_x;
			oy = Cost[y][x].from_y;
			circle(src, Point(x, y), 1, Scalar(0, 0, 255), -1);
			s.push(Point(x, y));
		}
		while (s.size())
		{
			pts.push_back(s.top());
			s.pop();
		}
		imshow("win_name", src);
		current = Point(x, y);	
}
void cut()
{
	Mat final = Mat::zeros(src.size(), CV_8UC3);
	Mat mask = Mat::zeros(src.size(), CV_8UC1);
	vector<vector<Point> > vpts;
	vpts.push_back(pts);
	fillPoly(mask, vpts, Scalar(255, 255, 255), 8, 0);
	
	bitwise_and(xx, xx, final, mask);
	imshow("Result", final);;
	imwrite("C:\\Users\\DELL\\Desktop\\5.png", final);
}
void mousehandle(int event, int x, int y, int flags, void* param) {
	if (safe) {
		if (event == CV_EVENT_MOUSEMOVE) 
		{
			cout << y << "  " << x << endl;
			line(x, y);
		}
	}
	if (event == CV_EVENT_LBUTTONDOWN) 
	{
		mark(x, y); 
		dij(x, y);
		safe ++;
	}
}
int main() 
{
	namedWindow("win_name", CV_WINDOW_AUTOSIZE);
	source = imread("C:\\Users\\DELL\\Desktop\\9.jpg");
	src = source;
	xx = imread("C:\\Users\\DELL\\Desktop\\9.jpg");
	Initial();
	initial_cost_node();
	imshow("win_name", source);
	setMouseCallback("win_name", mousehandle, (void*)& source);
	int key;
	key = cvWaitKey(10);
	while (1) {
		key = cvWaitKey(10);
		if (key == 27) break;
		switch (key)
		{
		case 'c':
		{
			cut();
			break;
		}
		}
	}
	waitKey(0);
	return 0;
}



