//#pragma once
//#define MAX_SIZE 100
//#include <iostream>
//class Graph
//{
//public:
//	int** Edge;//边，代表布线
//	int* Vertex;//代表设备的No
//	int NumEdge;
//	int NumVertex;
//	int MaxVertex;
//
//	Graph()
//	{
//		MaxVertex = MAX_SIZE;
//		NumEdge = NumVertex = 0;
//		Vertex = new int[MaxVertex];
//		Edge = new int* [MaxVertex];
//		for (int i = 0; i < MaxVertex; i++)
//		{
//			Edge[i] = new int[MaxVertex];
//		}
//		for (int i = 0; i < MaxVertex; i++)
//		{
//			for (int j = 0; j < MaxVertex; j++)
//			{
//				Edge[i][j] = 0;
//			}
//		}
//	}
//
//	//插入节点
//	void InsertVertex(int v)
//	{
//		if (NumVertex > MaxVertex)
//			return;
//		Vertex[NumVertex++] = v;
//	}
//
//	//查找一个顶点v
//	int GetVertexI(int v)
//	{
//		for (int i = 0; i < NumVertex; i++)
//		{
//			if (Vertex[i] == v)
//				return i;
//		}
//		return -1;
//	}
//
//	//插入一条由点v1指向v2的边
//	void InsertEdge(int v1, int v2)
//	{
//		int p1 = GetVertexI(v1);
//		int p2 = GetVertexI(v2);
//		if (p1 == -1 || p2 == -1)
//			return;
//		if (Edge[p1][p2] == 0)
//		{
//			Edge[p1][p2] = 1;
//			NumEdge++;
//		}
//	}
//
//	//打印函数
//	void ShowGraph()
//	{
//		int i, j;
//		std::cout << "  ";
//		for (i = 0; i < NumVertex; i++)
//			std::cout << Vertex[i] << " ";
//		std::cout << std::endl;
//		for (i = 0; i < NumVertex; i++)
//		{
//			std::cout << Vertex[i] << " ";
//			for (j = 0; j < NumVertex; j++)
//				std::cout << Edge[i][j] << " ";
//			std::cout << std::endl;
//		}
//	}
//
//	//获取某个节点的出度
//	int GetOutDegree(int v)
//	{
//		int p = GetVertexI(v);
//		if (p == -1)
//			return 0;
//		int n = 0;
//		for (int i = 0; i < NumVertex; i++)
//		{
//			if (Edge[p][i] == 1)
//				n++;
//		}
//		return n;
//	}
//	//获取某个节点的入度
//	int GetInDegree(int v)
//	{
//		int p = GetVertexI(v);
//		if (p == -1)
//			return 0;
//		int n = 0;
//		for (int i = 0; i < NumVertex; i++)
//		{
//			if (Edge[i][p] == 1)
//				n++;
//		}
//		return n;
//	}
//	//获取某个节点的总度数
//	int GetTotalDegree(int v)
//	{
//		return GetOutDegree(v) + GetInDegree(v);
//	}
//	////删除一个顶点
//	//void DeleteVertex(int v)
//	//{
//	//	int p = GetVertexI(v);
//	//	if (p == -1)
//	//		return;
//	//	int i, j;
//	//	int n = GetEdgeNum(v);
//	//	for (i = p; i < NumVertex - 1; i++)  //顶点先删除
//	//		Vertex[i] = Vertex[i + 1];
//
//
//	//	for (i = p; i < NumVertex - 1; i++)  //行上移
//	//	{
//	//		for (j = 0; j < NumVertex; j++)
//	//			Edge[i][j] = Edge[i + 1][j];
//	//	}
//	//	for (i = 0; i < NumVertex - 1; i++)  //列左移
//	//	{
//	//		for (j = p; j < NumVertex - 1; j++)
//	//		{
//	//			Edge[i][j] = Edge[i][j + 1];
//	//		}
//	//	}
//
//	//	NumVertex--;
//	//	NumEdge -= n;
//
//	//}
//	//void DeleteEdge(int v1, int v2)//删除顶点v1和v2之间的边
//	//{
//	//	int p1 = GetVertexI(v1);
//	//	int p2 = GetVertexI(v2);
//	//	if (p1 == -1 || p2 == -1)
//	//		return;
//	//	if (Edge[p1][p2] == 0)
//	//		return;
//	//	Edge[p1][p2] = Edge[p2][p1] = 0;
//	//	NumEdge--;
//	//}
//};