#ifndef QUICKSORT_H
#define QUICKSORT_H
#include <vector>
#include<iostream>
template<class T>
void quickSort(std::vector<T>& a,int low,int high)
{
    if(low >= high)
        {
            return;
        }
    int first = low;
    int last = high;
    //use the first element as pivot
    T pivot = a[low];
    //std::cout<<"low:"<<low<<" ,high:"<<high<<std::endl;
    while(first < last)
    {
        //find the first elemrnt from the bottom that <pivot
        while(first < last && a[last] >= pivot)
        {
            --last;
        }

        //find the first element from the front that > pivot
        while(first < last && a[first] <= pivot)
        {
            ++first;
        }
        //std::cout<<"first:"<<first<<" ,last:"<<last<<std::endl;
        //swithc them
        T tmp=a[first];
        a[first]=a[last];
        a[last]=tmp;

    }
    //relocate pivot
    a[low]=a[first];
    a[first] = pivot;
    //recursive quick sort both sight of pivot
    if(first>low)
        quickSort(a, low, first-1);
    if(first<high)
        quickSort(a, first+1, high);

}
#endif // QUICKSORT_H

