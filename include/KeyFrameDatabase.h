/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef KEYFRAMEDATABASE_H
#define KEYFRAMEDATABASE_H

#include <vector>
#include <list>
#include <set>

#include "KeyFrame.h"
#include "Frame.h"
#include "ORBVocabulary.h"

#include<mutex>


namespace ORB_SLAM2
{

class KeyFrame;
class Frame;


class KeyFrameDatabase
{
public:

    KeyFrameDatabase(const ORBVocabulary &voc);

   void add(KeyFrame* pKF);

   void erase(KeyFrame* pKF);

   void clear();

   // Loop Detection
   std::vector<KeyFrame *> DetectLoopCandidates(KeyFrame* pKF, float minScore);

   // Relocalization
   std::vector<KeyFrame*> DetectRelocalizationCandidates(Frame* F);

protected:

  // Associated vocabulary
  const ORBVocabulary* mpVoc; ///< 预先训练好的词典

  // Inverted file
  std::vector<list<KeyFrame*> > mvInvertedFile; ///< 倒排索引，mvInvertedFile[i]表示包含了第i个word id的所有关键帧

  // Mutex
  std::mutex mMutex;
};

} //namespace ORB_SLAM

#endif
/*值得学习的：
 * 1、选择回环检测的候选帧：1)找出与当前关键帧相连的所有关键帧，利用mvInvertedFile来统计当前关键帧与其他关键帧(不包括相连关键帧)的共有单词，记录最大共有单词数量m，将共有单词数小于0.8m的关键帧剔除
 *     2)记录候选闭环关键帧和当前关键帧的相似度分数，将小于minScore的候选关键帧剔除。
 *     3)找到与当每一个候选关键帧具有最好共视关系的十帧关键帧，并将其划分成一组，记录这一组中分数最高的那个关键帧以及该组的累计分数
 *     4)十组中累计分数最高的分数s乘上0.75作为阈值，将比该阈值高的那些组中分数最高的关键帧作为最终闭环候选帧
 * 2、重定位候选帧的筛选和回环检测的候选帧的筛选几乎完全一样，只不过，在此处，不需要剔除与当前帧相连的关键帧。