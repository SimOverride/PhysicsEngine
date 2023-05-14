#ifndef _stack_h
#define _stack_h

#include "boolean.h"

typedef struct node
{
	void* data;
	struct node* next;
}Node;

/// <summary>
/// 支持任意类型数据的链表，数据必须是指针
/// </summary>
typedef struct list
{
	Node* head;
	Node* tail;
	int dataSize;
	int length;
}List;

/// <summary>
/// 新建空链表
/// </summary>
/// <param name="dataSize">储存数据的大小</param>
/// <returns></returns>
List* CreateList(int dataSize);

/*添加数据*/
/// <summary>
/// 头插链表
/// </summary>
void InsertFromHead(List* list, void* data);
/// <summary>
/// 尾插链表
/// </summary>
void InsertFromTail(List* list, void* data);
void InsertByIndex(List* list, void* data, int index);

/*移除数据*/
bool RemoveFromHead(List* list);
bool RemoveFromTail(List* list);
bool RemoveByIndex(List* list, int index);
/// <summary>
/// 清空链表
/// </summary>
void ClearList(List* list);

void* GetData(List* list, int index);
bool IsContain(List* list, void* data);
bool IsEmpty(List* list);

#endif