# Graph
keypath and tuopo_sort of Graph
#include <malloc.h>
#include <stdio.h>
#include <stdlib.h>
#define Max 100
#define OK 1
#define ERROR 0   // 最大顶点数
int vertex_early[Max],vertex_late[Max],edge_early[Max],edge_late[Max];/*四个数组记录相应的
（事件）最早，最晚开始时间，活动最早，最晚开始时间; */
typedef struct Edgenode{
	int vertex;           //邻接点下标; 
	int weight;         //权值
	struct Edgenode *next;  //链域
}Edgenode;
typedef struct Vertexnode {   //顶点边结点
    int data;         //顶点域
    Edgenode *Head;//边表头指针
}Vertexnode,List[Max];
   //List是邻接表类型
typedef struct graph{   //图的结构
    List adjlist;       //邻接表
    int num_node,num_edge;            //图中当前顶点数和边数
}graph,*Graph;
typedef struct Seqstack{
	int top;
	int a[Max];
}Seqstack;
Seqstack *stack2;
void Initstack(Seqstack *S)
{
	S=(Seqstack *)malloc(sizeof(Seqstack));
	S->top=0;
}
void Pushstack(Seqstack *S,int e)
{
	S->top++;
	S->a[S->top]=e;
}
void Popstack(Seqstack *S)
{
	S->top--;
}
int Gettop(Seqstack S)
{
	return S.a[S.top];
}
int Isemptystack(Seqstack S)
{
	if(S.top==0)
		return OK;
	else
		return ERROR;
}

int  Location(graph G,int a)
{
    int i,j;
    for(i=0;i<G.num_node;i++)
    {
        if(G.adjlist[i].data==a)
            return i;
    }
}
void Creategraph(graph *G)
{	int i,j,k;
	G=(graph *)malloc(sizeof(graph)); 
	printf("请输入图的顶点数和弧的数目：\n"); 
	int a,b;
	scanf("%d%d",&G->num_node,&G->num_edge);
	printf("请输入图的顶点信息：\n");
	for(i=0;i<G->num_node;i++)
	{
		scanf("%d",&G->adjlist[i].data);
		G->adjlist[i].Head=NULL;
	}
	printf("请输入边的信息弧尾和弧头的序列以及弧所对应的权值:\n");
	for(i=0;i<G->num_edge;i++)
	{
		int v1,v2,v3;
		Edgenode *temp;
		temp=(Edgenode *)malloc(sizeof(Edgenode));
		scanf("%d%d%d",&v1,&v2,&v3);
		temp->vertex=v2;
		temp->weight=v3;
		temp->next=G->adjlist[Location(*G,v1)].Head;
		G->adjlist[Location(*G,v1)].Head=temp;	
	 }  
	 printf("创建成功：\n");
}
/*void Showdegree(graph G)
{	printf("各顶点的入度为：\n");
	int i;
	for(i=0;i<G.num_node;i++)
	{
		printf("%d ",G.adjlist[i].in);
	}
}*/
void Degree(graph G,int indegree[])
{
	int i;
	for(i=0;i<G.num_node;i++)
	{
		indegree[i]=0;//先初始化各个顶点的入度； 
	}
	for(i=0;i<G.num_node;i++)
	{
		Edgenode *temp;
		temp=G.adjlist[i].Head;
		while(temp)
		{
			indegree[temp->vertex]++;
			temp=temp->next;	
		}	
	}
	
}//求各个顶点的入度； 
void Toposort(graph G)
{	int indegree[Max];
	Degree(G,indegree);
	int i,j,Nodecount=0;
	Edgenode *temp;
	Seqstack *stack1;
	Initstack(stack1);
	for(i=0;i<G.num_node;i++)
	{
		if(indegree[i]==0)
			Pushstack(stack1,i);
	}
	printf("拓扑序列为：\n"); 
	while(!Isemptystack(*stack1))
	{	
		int top=Gettop(*stack1);
		Popstack(stack1);
		Pushstack(stack2,top);
		//找到出栈元素所对应的下标;
		printf("V%d",top);
		Nodecount++;
		temp=G.adjlist[top].Head;
		while(!temp)
		{
			indegree[temp->vertex]--;
			if(indegree[temp->vertex]==0)
				Pushstack(stack1,temp->vertex);
			if(vertex_early[top]+temp->weight>vertex_early[temp->vertex])
				vertex_early[temp->vertex]=vertex_early[top]+temp->weight;/*如果边的源点的最长路径长度加上边的权值比
				汇点的最长路径长度还长，就覆盖ve数组中对应位置的值，最终结束
				时，ve数组中存储的就是各顶点的最长路径长度。*/ 
			temp=temp->next;
			
		 } 
	}
	if(Nodecount<G.num_node)
		printf("图中有环：\n"); 
}
void Keypath(graph G)
{
	int i,j,k;
	Toposort(G);
	Edgenode *p;
	for(i=0;i<G.num_node;i++)
	{
		vertex_late[i]=vertex_early[G.num_node-1];//将汇点的最早发生时间赋值给最晚发生时间； 
	}
	while(!Isemptystack(*stack2))
	{	j=Gettop(*stack2);
		Popstack(stack2);
		for(p=G.adjlist[j].Head;p;p=p->next)
		{	k=p->vertex;
			if(vertex_late[k]-p->weight<vertex_late[j])
				vertex_late[j]=vertex_late[k]-p->weight;
		}
		
		
	}
	for(j=0;j<G.num_node;j++)
	{
		for(p=G.adjlist[j].Head;p;p=p->next)
		{
			k=p->vertex;
			if(vertex_early[i]==vertex_late[k]-p->weight)
			printf("从V%d到V%d是关键路径\n",j+1,k+1);
            else
            printf("从V%d到V%d不是关键路径\n",j+1,k+1);
		}
	}
	
}
int main()
{	graph *G1;
	G1=(graph *)malloc(sizeof(graph));
	Creategraph(G1);
	
	Toposort(*G1);
	Keypath(*G1);
	return 0;
}
