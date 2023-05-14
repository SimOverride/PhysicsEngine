#include <stdio.h>
#include <stdlib.h>

#include "list.h"
#include "vector.h"

List* CreateList(int dataSize)
{
	List* list = (List*)malloc(sizeof(List));
	list->head = NULL;
	list->tail = NULL;
	list->dataSize = dataSize;
	list->length = 0;
	return list;
}

void InsertFromHead(List* list, void* data)
{
	Node* newNode = (Node*)malloc(sizeof(Node));
	newNode->data = data;

	list->length++;
	if (IsEmpty(list))
	{
		newNode->next = NULL;
		list->head = list->tail = newNode;
		return;
	}
	
	newNode->next = list->head;
	list->head = newNode;
}

void InsertFromTail(List* list, void* data)
{
	if (data == NULL)
		return;

	Node* newNode = (Node*)malloc(sizeof(Node));
	newNode->data = data;

	list->length++;
	if (IsEmpty(list))
	{
		newNode->next = NULL;
		list->head = list->tail = newNode;
		return;
	}

	newNode->next = NULL;
	list->tail->next = newNode;
	list->tail = newNode;
}

void InsertByIndex(List* list, void* data, int index)
{
	Node* newNode = (Node*)malloc(sizeof(Node));
	newNode->data = data;

	if(index <= 0)
	{
		InsertFromHead(list, data);
		return;
	}

	if(index >= list->length)
	{
		InsertFromTail(list, data);
		return;
	}

	Node* t = list->head;
	int i;
	for(i = 0;i < index - 1;i++)
	{
		t = t->next;
	}
	newNode->next = t->next;
	t->next = newNode;
	list->length++;
}

bool RemoveFromHead(List* list)
{
	if (IsEmpty(list))
		return FALSE;

	Node* t = list->head;
	list->head = list->head->next;
	if (list->length == 1)
		list->tail = list->head;
	free(t);
	list->length--;

	return TRUE;
}

bool RemoveFromTail(List* list)
{
	if (IsEmpty(list))
		return FALSE;
	
	if (list->length == 1)
	{
		list->head = list->head->next;
		free(list->tail);
		list->head = list->tail = NULL;
		list->length--;
		return TRUE;
	}
		
	Node* first = list->head;
	while(first->next != list->tail)
	{
		first = first->next;
	}
	Node* t = list->tail;
	first->next = NULL;
	list->tail = first;
	free(t);
	list->length--;

	return TRUE;
}

bool RemoveByIndex(List* list, int index)
{
	if (index <= 0)
	{
		return RemoveFromHead(list);
	}

	if (index >= list->length - 1)
	{
		return RemoveFromTail(list);
	}

	Node* first = list->head;
	int i;
	for (i = 0; i < index - 1; i++)
	{
		first = first->next;
	}
	Node* t = first->next;
	first->next = first->next->next;
	free(t);
	list->length--;

	return TRUE;
}

void ClearList(List* list)
{
	while (!IsEmpty(list))
		RemoveFromTail(list);
}

void* GetData(List* list, int index)
{
	if (IsEmpty(list))
		return NULL;

	if (index <= 0)
		return list->head->data;
	
	if (index >= list->length - 1)
		return list->tail->data;

	Node* t = list->head;
	int i;
	for (i = 0; i < index; i++)
	{
		t = t->next;
	}
	return t->data;
}

bool IsContain(List* list, void* data)
{
	if(list->dataSize == sizeof(Vector))
	{
		int i;
		for(i = 0;i < list->length;i++)
		{
			Vector v = *(Vector*)GetData(list, i);
			if (IsVectorsEqual(v, *(Vector*)data))
				return TRUE;
		}
	}

	return FALSE;
}

bool IsEmpty(List* list)
{
	if (list->head == NULL)
		return TRUE;
	return FALSE;
}