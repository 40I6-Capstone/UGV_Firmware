#include "../MemFilt.hpp"
#include <iostream>
#include <cstring>

template <typename T, int size>
class MedianFilt : public MemFilt<T, size>
{
private:
    T calculate();
    void sort(T *arr);

public:
    void update(T val);
    MedianFilt();
};

template <typename T, int size>
MedianFilt<T, size>::MedianFilt()
{
}

template <typename T, int size>
void MedianFilt<T, size>::update(T val)
{
    MemFilt<T, size>::shift(val);
    // for (size_t i = 0; i < size; i++)
    // {
    //     std::cout << this->m_taps[i] << " ";
    // }
    // std::cout << "sort-> ";
    this->calculate();
}

template <typename T, int size>
T MedianFilt<T, size>::calculate()
{

    // Create space for sorted array
    T sorted_arr[size];
    memcpy(sorted_arr, this->m_taps, sizeof(this->m_taps));
    this->sort(sorted_arr);
    // for (size_t i = 0; i < size; i++)
    // {
    //     std::cout << sorted_arr[i] << " ";
    // }

    if (size % 2 == 0)
    {
        this->m_output = (sorted_arr[size / 2] + sorted_arr[size / 2 + 1]) / 2;
    }
    else
    {
        this->m_output = (sorted_arr[size / 2]);
    }
    return this->m_output;
}

template <typename T, int size>
void MedianFilt<T, size>::sort(T *arr)
{
    // Not using std::sort b/c of possible recursion/heap usage, trying to avoid that on an embedded chip, not that I know if it will actually cause a problem
    for (int i = 1; i < size; i++)
    {
        T x = arr[i];
        int j = i - 1;
        while ((j >= 0) && (arr[j] > x))
        {
            arr[j + 1] = arr[j];
            j--;
        }
        arr[j + 1] = x;
    }
}