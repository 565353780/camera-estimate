#ifndef KDTREE_H
#define KDTREE_H
#include<vector>
#include<stdlib.h>
#include<iostream>
#include"quicksort.h"
#include "Math/MathDefines.h"
#define INFINITY 10000.0
namespace KDTree
{
//    template<class KDPoint>
//    void linearKNN(int k, KDPoint point,std::vector<KDPoint> dataSet,std::vector<double> &distanceSet, std::vector<int> &nodeSet)
//    {
//        if(distanceSet.size()<k)
//        {
//            for(int i=0;i<k; i++)
//            {
//                distanceSet.push_back(INFINITY);
//                nodeSet.push_back(0);

//            }
//        }
//        for(int node=0; node<dataSet.size(); node++)
//        {
//            double distance=(dataSet[node]-point).norm();
//            maintainKNearest(distanceSet,nodeSet,node,distance);
//        }
//    }

    template<int D>
    class KDTree
    {
        typedef GCL::Vec<D,double> KDPoint;
    public:
        struct Node
        {
            Node* leftChild_;
            Node* rightChild_;
            Node* parent_;
            KDPoint data_;
            KDPoint range_low_;
            KDPoint range_high_;
            int layer_;
            int splitDimension_;
            Node(Node* parent,KDPoint data,KDPoint rangeLow, KDPoint rangehigh, int layer, int splitdimension)
            {
                data_=data; range_low_=rangeLow; range_high_=rangehigh;
                leftChild_=NULL; rightChild_=NULL; parent_=parent;
                layer_=layer; splitDimension_=splitdimension;
            }
        };

        double variance(int d, std::vector<KDPoint> dataset)
        // 计算单维度方差
        {
            double s=0;
            double t=0;
            int n=dataset.size();
            for(int i=0;i<n;i++)
            {
                s+=dataset[i][d];
                t+=dataset[i][d]*dataset[i][d];
            }

            return t*t/n-s*s/n/n;
        }

        int medium(int dimension, std::vector<KDPoint> dataset)
        //找出单维度的中位数
        {
            std::vector<double> temp;
            for(int i=0; i<dataset.size(); i++)
            {
                temp.push_back(dataset[i][dimension]);
            }
            std::vector<double> temp2=temp;
            quickSort<double>(temp2,0,temp2.size()-1);
            double mediumVal=temp2[temp2.size()/2];
            for(int i=0; i<temp.size(); i++)
            {
                if(temp[i]==mediumVal)
                {
                    return i;
                }
            }
        }

        Node* root_;
        int dimension_;
        KDTree(std::vector<KDPoint> dataset, KDPoint lowerlimit, KDPoint upperlimit,int dim=0)
        {

            root_=CreateKDTree(NULL,dataset,lowerlimit,upperlimit,0,dim);

        }

        Node* CreateKDTree(Node* parent,std::vector<KDPoint> &dataset, KDPoint lowwerlimit, KDPoint upperlimit, int layer,int dim=0)
        //从某节点建立K-D树
        {
            //std::cout<<layer<<std::endl;
            if(dataset.size()==0)
                return NULL;
            if(dim==0)
                dimension_=lowwerlimit.size();
            else
                dimension_=dim;
            int splitDimension=0;
            double variance_max=0;
            //找出方差最大的维度
            for(int i=0; i<dimension_; i++)
            {
                double result=variance(i,dataset);
                if(variance_max<result)
                {
                    splitDimension=i;
                      variance_max=result;
                }
            }
            std::vector<KDPoint> datasetLeft;
            std::vector<KDPoint> datasetRight;
            int nodeNum=medium(splitDimension,dataset);
            double midval=dataset[nodeNum][splitDimension];
            // 把选点节点外的点放入左右子集并确定左右范围
            for (int i=0; i<dataset.size(); i++)
            {
                if(dataset[i][splitDimension]<midval)
                {datasetLeft.push_back(dataset[i]);}
                else if(i!=nodeNum)
                {datasetRight.push_back(dataset[i]);}
            }
            KDPoint leftUpper=upperlimit;
            KDPoint rightLower=lowwerlimit;
            leftUpper[splitDimension]=midval;
            rightLower[splitDimension]=midval;

            //记录当前节点的信息
            Node *pNode=new Node(parent,dataset[nodeNum],lowwerlimit,upperlimit, layer, splitDimension);
            pNode->parent_=parent;

            //递归定义左右节点的KD树

            pNode->leftChild_=CreateKDTree(pNode,datasetLeft,lowwerlimit,leftUpper, layer+1,dim);
            pNode->rightChild_=CreateKDTree(pNode,datasetRight,rightLower,upperlimit,layer+1,dim);


            return pNode;
        }
        void travelPrint(Node* node, int &i)
        {


            if(node)
            {
                //std::cout<<"("<<node->layer_<<","<<node->splitDimension_<<","<<i<<"|"<<node->data_<<")"<<std::endl;
                i+=1;
                travelPrint(node->leftChild_,i);
                travelPrint(node->rightChild_,i);

            }
        }
        void PrintTree()
        {
            int i=0;
            travelPrint(root_,i);
        }
        void maintainKNearest(std::vector<double> &nearest, std::vector<Node*> &nearestNode , Node* node, double dis)
        //把新节点放入距离集中的合适位置
        {
            int n=nearest.size();
            if(dis<nearest.back())
            {
                nearest[n-1]=dis;
                nearestNode[n-1]=node;
                int i=n-1;
                while(nearest[i]<nearest[i-1] && i>0)
                {

                    nearest.push_back(nearest[i-1]);
                    nearest[i-1]=nearest[i];
                    nearest[i]=nearest.back();
                    nearest.pop_back();
                    nearestNode.push_back(nearestNode[i-1]);
                    nearestNode[i-1]=nearestNode[i];
                    nearestNode[i]=nearestNode.back();
                    nearestNode.pop_back();
                    i-=1;
                }
            }
        }

        void findKNearest(Node* inputnode, int k, KDPoint point,std::vector<double> &distanceSet, std::vector<Node*> &nodeSet)
        //找出输入点为根的K-D树中最邻近的k个点
        {
            //std::cout<<"MainFun"<<std::endl;
            //j距离集初始化
            if(distanceSet.size()<k)
            {
                for(int i=0;i<k; i++)
                {
                    distanceSet.push_back(INFINITY);
                    nodeSet.push_back(NULL);

                }
            }
            Node* currentNode=inputnode;
            //寻找近似最近叶子节点

            while(currentNode!=NULL)
            {
                //std::cout<<"Forward"<<std::endl;
                int splitDimension=currentNode->splitDimension_;
                //std::cout<<"("<<currentNode->layer_<<","<<currentNode->splitDimension_<<"|"<<currentNode->data_<<")"<<std::endl;
                int i;
                //std::cin>>i;
                double distance=(currentNode->data_-point).norm();
                maintainKNearest(distanceSet,nodeSet,currentNode, distance);//更新距离集
                double v0=point[splitDimension], v1=(currentNode->data_)[splitDimension];
                if(v0<v1)
                {
                    if(!currentNode->leftChild_)
                        break;
                    //std::cout<<"left"<<std::endl;
                    currentNode=currentNode->leftChild_;
                }
                else
                {
                    if(!currentNode->rightChild_)
                        break;
                    //std::cout<<"right"<<std::endl;
                    currentNode=currentNode->rightChild_;
                }
            }



            double distance;
            //回溯并更新距离集
            while(1)
            {
                int i;
//                std::cout<<"Back"<<std::endl;
//                std::cin>>i;

                if(currentNode==inputnode)// 回溯到根节点时停止
                    break;


                Node* parent=currentNode->parent_;
                //std::cout<<"parent:"<<parent->data_<<std::endl;
                Node* sibling;
                //确定当前节点是左节点还是右节点并计算目标点到分隔超平面距离
                int d=parent->splitDimension_;
                double disToBound;
                if (currentNode==parent->leftChild_)
                {
                    //std::cout<<"leftchild"<<std::endl;
                    disToBound=currentNode->range_high_[d]-point[d];
                    sibling=parent->rightChild_;
                }
                else
                {
                    //std::cout<<"rightchild"<<std::endl;
                    disToBound=point[d]-currentNode->range_high_[d];
                    sibling=parent->leftChild_;
                }
                if(disToBound>distanceSet.back() || currentNode==root_)//距离集中最大距离小于点到超平面距离，回溯停止
                {
                    return;
                }
                else
                {
                    if(sibling)
                    {
                        //std::cout<<"calling sibling"<<std::endl;
                        findKNearest(sibling,k,point,distanceSet,nodeSet); //递归访问兄弟节点树更新距离集
                    }
                    currentNode=currentNode->parent_;
                }

            }
        }
    };

}
#endif // KDTREE_H
