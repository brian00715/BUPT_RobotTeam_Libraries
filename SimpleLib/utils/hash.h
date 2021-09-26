/*******************************************************************************
 * Copyright:		BUPT
 * File Name:		hash.h
 * Author:			wangzhi
 * Description:		None
 * Bug:				None
 * Version:			0.1
 * Data:			2017/03/28
 * History          Modified 2019/09/20 ZeroVoid
 * Todo:			None
 *******************************************************************************/

#ifndef __HASH_H
#define __HASH_H

#ifdef __cplusplus
extern "C" {
#endif

#define BUCKET_SIZE 31

typedef struct HashNode{
	const void * key;
	void * value;
	struct HashNode * next;
} HashNode;

typedef struct{
	HashNode ** bucket;
	int length;

	/* 用来实现HashTable_Map必须强制实施的一项已检查的运行时错误：map函数遍历表项是不能改变表的
	 * 可以改变表项中的值，但是不能插入或删除表项
	 * timestamp在insert和remove时加一
	 * map函数在遍历后使用assert检查前后是否一致。
	 */
	unsigned int timestamp;

	int (*cmp)(const void *x, const void * y);
	unsigned int (*hash)(const void *key);
	void (*freeKey)(const void *key);
}* HashTable;  // HashTable是指向结构体的指针

unsigned int HashStr(const void *key);

HashTable HashTable_Create(int (*cmp)(const void *x, const void * y),unsigned int (*hash)(const void *key),void (*freeKey)(const void *key));

void HashTable_Destory(HashTable * hashTable);

int HashTable_GetLength(HashTable hashTable);

/*
 * @brief 插入一个表项
 *
 * @param HashTable  hashTable  哈希表
 *        const void *key       键
 *        void * value          值
 *
 * @return 若之前有过该值，则覆盖之前的值，返回之前的值，否则返回NULL
 */
void * HashTable_Insert(HashTable  hashTable, const void *key, void * value);

/*
 * @brief 获取值
 *
 * @param HashTable  hashTable  哈希表
 *        const void *key       键
 *
 * @return 查找key对应的值，查找到，返回查找到的值，否则返回NULL
 */
void * HashTable_GetValue(HashTable hashTable,const void *key);

/*
 * @brief 删除某键值对
 *
 * @param HashTable  hashTable  哈希表
 *        const void *key       键
 *
 * @return 查找key对应的值，查找到，删除并返回查找到的键值对，否则返回NULL
 */
void * HashTable_Remove(HashTable hashTable, const void *key);

void HashTable_Map(HashTable hashTable, void (*apply)(const void *key, void **value, void *c1), void *c1);

void **HashTable_ToArray(HashTable table, void *end);

#ifdef __cplusplus
}
#endif

#endif /* __HASH_H */