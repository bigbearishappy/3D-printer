#include <stdio.h>
#include <string.h>

int* findDisappearedNumbers(int* nums, int numsSize, int* returnSize) 
{
	int i,j = 0;
    int *p = (int *)malloc(numsSize);
    for(i = 0;i < numsSize; i++)
    	p[i] = 0;
    for(i = 0;i < numsSize; i++)
    	p[nums[i] - 1]++;

    for(i = 0;i < numsSize; i++)
    	if(!p[i])
    		nums[j++] = i;

    *returnSize = j;

    return nums;
}

void main(void)
{
    int arr[8] = {4,3,2,7,8,2,3,1};
    int *p;
    int res = 0,i;
    p = findDisappearedNumbers(arr, sizeof(arr)/sizeof(arr[0]),&res);
    for(i = 0;i < res; i++)
    	printf("%d ",p[i]);

    while(1);
}