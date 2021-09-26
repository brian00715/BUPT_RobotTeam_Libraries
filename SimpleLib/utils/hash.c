/*******************************************************************************
 * Copyright:		BUPT
 * File Name:		hash.c
 * Author:			wangzhi
 * Description:		None
 * Bug:				None
 * Version:			0.1
 * Data:			2019/09/20 Fri 20:15
 * History:         Modified 2019/09/20 by ZeroVoid
 * Todo:			None
 *******************************************************************************/
#include "hash.h"
#include <assert.h>
#include <limits.h>
#include <stdlib.h>

static int cmpAtom(const void *x, const void *y)
{
    return x != y;
}

static unsigned long hashAtom(const void *key)
{
    return (unsigned long)key >> 2;
}

static void freeKeyAtom(const void *key)
{
}

unsigned int HashStr(const void *key)
{
    unsigned int hash = 5381;
    char *ch = (char *)key;
    while (*ch)
    {
        hash += (hash << 5) + (*ch++);
    }
    return (hash & 0x7FFFFFFF);
}

HashTable HashTable_Create(int (*cmp)(const void *, const void *),
                           unsigned int (*hash)(const void *),
                           void (*freeKey)(const void *key))
{
    int i;
    HashTable hashTable;
    //确定bucket的数目，分配内存，bucket内存紧跟着hashTable
    hashTable = malloc(sizeof(*hashTable) + BUCKET_SIZE * sizeof(HashNode));

    //初始化
    hashTable->cmp = cmp ? cmp : cmpAtom;
    hashTable->hash = hash ? hash : (unsigned int (*)(void const *))hashAtom;
    hashTable->freeKey = freeKey ? freeKey : freeKeyAtom;
    hashTable->bucket = (HashNode **)(hashTable + 1);
    for (i = 0; i < BUCKET_SIZE; ++i)
        hashTable->bucket[i] = NULL;
    hashTable->length = 0;
    hashTable->timestamp = 0;

    return hashTable;
}

void HashTable_Destory(HashTable *hashTable)
{
    assert(hashTable && *hashTable);

    if ((*hashTable)->length > 0)
    {
        HashNode *p, *q;
        for (int i = 0; i < (*hashTable)->length; ++i)
        {
            for (p = (*hashTable)->bucket[i]; p; p = q)
            {
                q = p->next;
                ((*hashTable)->freeKey)(p->key);
                free(p);
            }
        }
    }
    free(hashTable);
}

int HashTable_GetLength(HashTable hashTable)
{
    return hashTable->length;
}

/**
 * @brief 将键值对添加到哈希表中
 * @param hashTable 哈希表数组
 * @param key 键
 * @param value key所对应的值
 **/
void *HashTable_Insert(HashTable hashTable, const void *key, void *value)
{
    void *prev = NULL; //之前的值
    HashNode *p;
    unsigned int index;

    assert(hashTable);
    assert(key);

    //search hashTable for key
    index = hashTable->hash(key) % BUCKET_SIZE; // 计算存储位置
    //printf("insert index:%d\n", index);
    for (p = hashTable->bucket[index]; p; p = p->next)
    {
        if (hashTable->cmp(key, p->key) == 0) // 如果重名，则直接return
        {
            break;
        }
    }

    if (p == NULL) // 如果存储位置重叠了，并且遍历到了链表的表尾，则将新元素添加到链表中
    {
        p = malloc(sizeof(*p));
        if (p == NULL)
        {
            return NULL;
        }
        p->key = key;
        p->next = hashTable->bucket[index];
        hashTable->bucket[index] = p;
        hashTable->length++;
    }
    else
    {
        prev = p->value;
    }

    p->value = value;
    hashTable->timestamp++;

    return prev;
}

void *HashTable_GetValue(HashTable hashTable, const void *key)
{
    unsigned int index;
    HashNode *p;

    assert(hashTable);
    assert(key);

    index = hashTable->hash(key) % BUCKET_SIZE;
    //printf("get index:%d\n", index);
    for (p = hashTable->bucket[index]; p; p = p->next)
    {
        if (hashTable->cmp(key, p->key) == 0)
        {
            break;
        }
    }

    return p ? p->value : NULL;
}

void *HashTable_Remove(HashTable hashTable, const void *key)
{
    HashNode **pp;
    unsigned int index;

    assert(hashTable);
    assert(key);

    //printf("key:%d\n", *(int *) key);

    hashTable->timestamp++;
    index = (hashTable->hash)(key) % BUCKET_SIZE;
    //printf("index:%d\n", index);
    for (pp = &hashTable->bucket[index]; *pp; pp = &((*pp)->next))
    {
        if ((hashTable->cmp)(key, (*pp)->key) == 0)
        {
            HashNode *p = *pp;
            void *value = p->value;
            *pp = p->next;
            (hashTable->freeKey)(p->key);
            free(p);
            hashTable->length--;
            return value;
        }
    }

    return NULL;
}

/**
 * @brief 遍历哈希表
 * @param hashTable 哈希表
 * @param apply 函数指针
 **/
void HashTable_Map(HashTable hashTable, void (*apply)(const void *key, void **value, void *c1), void *c1)
{
    HashNode *p;
    unsigned int stamp;

    assert(hashTable);
    assert(apply);

    stamp = hashTable->timestamp;
    // 多个key算出的哈希值相同时，会以链表的形式在哈希表元素中存储
    for (int i = 0; i < BUCKET_SIZE; ++i) // 遍历链表
    {
        for (p = hashTable->bucket[i]; p; p = p->next)
        {
            // p->key就是cmd_name;p->value就是cmd_usage,是一个字符串数组
            apply(p->key, &(p->value), c1); // 相当于_cmd_help(p->key, &(p->value), c1)
            assert(stamp == hashTable->timestamp);
        }
    }
}

void **HashTable_ToArray(HashTable hashTable, void *end)
{
    int i, j = 0;
    void **array;
    HashNode *p;
    assert(hashTable);
    array = malloc((2 * hashTable->length + 1) * sizeof(*array));
    for (i = 0; i < BUCKET_SIZE; i++)
        for (p = hashTable->bucket[i]; p; p = p->next)
        {
            array[j++] = (void *)p->key;
            array[j++] = p->value;
        }
    array[j] = end;
    return array;
}
