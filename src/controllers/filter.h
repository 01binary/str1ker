/*
                                                                                     ███████                  
 ████████████  ████████████   ████████████       █  █████████████  █           █  ███       ███  ████████████ 
█              █ █           █            █    █ █  █              █        ███      ███████    █            █
 ████████████  █   █         █████████████   █   █   █             █   █████      ███       ███ █████████████ 
             █ █     █       █            █      █    █            ████      █                  █            █
 ████████████  █       █     █            █      █      █████████  █          █   ███       ███ █            █
                                                                                     ███████                  
 filter.h

 Analog signal filter
 Created 09/20/2023

 Copyright (C) 2023 Valeriy Novytskyy
 This software is licensed under GNU GPLv3
*/

/*----------------------------------------------------------*\
| Includes
\*----------------------------------------------------------*/

#include <map>
#include <vector>

/*----------------------------------------------------------*\
| Namespace
\*----------------------------------------------------------*/

namespace str1ker {

/*----------------------------------------------------------*\
| filter class
\*----------------------------------------------------------*/

class filter
{
private:
  // Sample with the smallest number of occurrences
  int m_min;

  // Smallest number of occurences
  int m_minCount;

  // Sample with the largest number of occurrences
  int m_max;

  // Largest number of occurrences
  int m_maxCount;

  // Bins of sample values and the number of occurences of each
  std::map<int, int> m_bins;

  // Difference between incoming value and last value that resets bins
  int m_threshold;

  // How many samples to average on the way out for additional smoothing
  int m_average;

  // Buffer for computing a moving average of the sample with highest count
  std::vector<int> m_buffer;

  // The next circular buffer index to fill in
  int m_next;

public:
  //
  // Constructor
  //

  filter(int threshold, int average);

  //
  // Sampling
  //

  int operator() (int sample);
  bool isStable() const;
  void debug() const;

private:
  int average(int output);
};

} // namespace str1ker
