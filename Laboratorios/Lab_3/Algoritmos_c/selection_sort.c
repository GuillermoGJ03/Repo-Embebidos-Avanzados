#include <stdio.h>
#include <stdlib.h>

void swapped(int *v1, int *v2){
    int swap = *v1;
    *v1 = *v2;
    *v2 = swap;
}

void selectionSort(int *arr, int size){
    for(int i = 0; i < size-1; i++){
        int min_idx = i;
        for(int j = i + 1; j < size; j++){
            if(arr[j] < arr[min_idx]){
                min_idx = j;
            }
        }
        if(i != min_idx){
            swapped(&arr[i], &arr[min_idx]);
        }
    }
}

void bubbleSort(int *arr, int size){
    for(int i = 0; i < size-1; i++){
        for(int j = 0; j < size-1-i; j++){
            if(arr[j] > arr[j+1]){
                swapped(&arr[j], &arr[j+1]);
            }
        }
    }
}

int sequentialSearch(int *arr, int size, int value){
    if(size < 0) return -1;
    for(int i = 0; i < size; i++){
        if(arr[i] == value){
            return i;
        }
    }
    return -1;
}

int binarySearch(int *arr, int size, int value){
    int min = 0;
    int max = size - 1;
    int found = 0;

    while(found == 0){
        if (min > max){
            return -1;
        }

        int mid = (min + max)/2;
        if(arr[mid] == value){
            found = 1;
            return mid;
        } else if(arr[mid] < value){
            min = mid + 1;
        } else{
            max = mid - 1;
        }
    }
}

void print(int *arr, int size){
    for(int i = 0; i < size; i++){
        printf("%d ", arr[i]);
    }
    printf("\n");
}

int main(){

    int array[] = {35, 32, 0, 12, 1, 15, 25, -4, -2, -8};

    print(array, sizeof(array)/sizeof(array[0]));

    printf("%d ", sequentialSearch(array, sizeof(array)/sizeof(array[0]), 15));

    printf("\n");

    selectionSort(array, sizeof(array)/sizeof(array[0]));

    print(array, sizeof(array)/sizeof(array[0]));

    printf("%d ", sequentialSearch(array, sizeof(array)/sizeof(array[0]), 6));

    printf("\n");
    printf("\n");


    int array_1[] = {35, 32, 0, 12, 1, 15, 25, -4, -2, -8};

    print(array_1, sizeof(array_1)/sizeof(array_1[0]));

    bubbleSort(array_1, sizeof(array_1)/sizeof(array_1[0]));

    print(array_1, sizeof(array_1)/sizeof(array_1[0]));

    printf("%d ", binarySearch(array_1, sizeof(array_1)/sizeof(array_1[0]), 15));

    printf("\n"); 

    printf("%d ", binarySearch(array_1, sizeof(array_1)/sizeof(array_1[0]), 6));

    return 0;

}