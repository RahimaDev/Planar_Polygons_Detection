

#ifndef MSAC_H
#define MSAC_H

#include <vector>
#include <random>
#include <numeric>
#include <functional>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <queue>
#include <assert.h>

#include <omp.h>

namespace robest
{

template<typename T>
class MedianFinder {
    std::priority_queue<T> lo;                                    // max heap
    std::priority_queue<T, std::vector<T>, std::greater<T>> hi;   // min heap

public:
    void add(T num)
    {
        lo.push(num);

        hi.push(lo.top());
        lo.pop();

        if (lo.size() < hi.size()) {
            lo.push(hi.top());
            hi.pop();
        }
    }

    double median()
    {
        return lo.size() > hi.size() ? (double) lo.top() : (lo.top() + hi.top()) * 0.5;
    }
};

class EstimationProblem
{
public:
    // Functions to overload in your class:
    virtual double estimErrorForSample(int i) = 0;
    virtual void   estimModelFromSamples(const std::vector<int> & samplesIdx) = 0;
    virtual int    getTotalNbSamples() const = 0;

    int getNbParams() const { return nbParams; }
    int getNbMinSamples() const { return nbMinSamples; }

protected:
    void setNbParams(int i) { nbParams = i; }
    void setNbMinSamples(int i) { nbMinSamples = i; }

private:
    int nbParams = -1;
    int nbMinSamples = -1;
};

static std::random_device rd;  // random device engine, usually based on /dev/random on UNIX-like systems
static std::mt19937 rng(rd()); // initialize Mersennes' twister using rd to generate the seed

// Base class for Robust Estimators
class AbstractEstimator
{
public:
    // Generate X !different! random numbers from 0 to N-1
    // X - minimal number of samples
    // N - total number of samples
    // generated numbers are indices of data points

    std::vector<int> randomSampleIdx()
    {
        int minNbSamples = problem->getNbMinSamples();
        int totalNbSamples = problem->getTotalNbSamples();

        std::vector<int> allIdx(totalNbSamples);
        std::iota(std::begin(allIdx), std::end(allIdx), 0); // Fill with 0, 1, ..., totalNbSamples-1.

        // shuffle the elements of allIdx : Fisher–Yates shuffle
        for (int i = 0; i < minNbSamples; i++)
        {
            std::uniform_int_distribution<int> dist(0, totalNbSamples - i - 1);
            int randInt = dist(rng);
            std::swap(allIdx[totalNbSamples - i - 1], allIdx[randInt]);
        }

        //take last <minNbSamples> elements
        std::vector<int> idx(allIdx.end() - minNbSamples, allIdx.end());

        return idx;
    }

    int calculateIterationsNb(const int batchSize, const float alpha = 0.99f, const float gamma = 0.80f)
    {
        assert(batchSize > 0 && "The size of input data must be > 0");
        assert((alpha > 0.0f && alpha <  1.0f) && "Accepted value of success probability (aplha) is in range (0,1)");
        assert((gamma > 0.0f && gamma <= 1.0f) && "Accepted value of inlier's ratio (gamma) is in range (0,1]");

        if (gamma == 1.0f)
            return 1;

        // Counting the number of iteration
        int nbIter = 1 + int(std::log(1.0 - alpha) / std::log(1.0 - std::pow(gamma, batchSize)));
        if ( nbIter < 0 || nbIter > 50000) nbIter = 50000;

        assert(nbIter > 0 && "Number of interations must be > 0: check solver params or define nbIter manually");
        
        return nbIter;
    }

    double getInliersFraction() const { return inliersFraction; }
    std::vector<int> getInliersIndices() const { return inliersIdx; }

protected:
    void getInliers(double thres)
    {
        int totalNbSamples = problem->getTotalNbSamples();
        inliersIdx.clear();
#pragma omp parallel for
        for (int j = 0; j < totalNbSamples; ++j)
        {
            double error = problem->estimErrorForSample(j);
            
#pragma omp critical
            {
                if ((error * error) < (thres * thres))
                {
                    inliersIdx.push_back(j);
                }
            }
        }
        this->inliersFraction = (double)(inliersIdx.size()) / (double)(totalNbSamples);
    }

    std::shared_ptr<EstimationProblem> problem;
    std::vector<int> bestIdxSet;
    std::vector<int> inliersIdx;
    double inliersFraction = -1.0;

};


class MSAC : public AbstractEstimator
{
public:
    MSAC()
    {
        sumSqErr = std::numeric_limits<double>::max();
    }

    void solve(std::shared_ptr<EstimationProblem> pb, double thres
               , int nbIter)
    {
        problem = pb;

        int totalNbSamples = problem->getTotalNbSamples();

        //#pragma omp parallel for
        for (int i = 0; i < nbIter; ++i)
        {
            std::vector<int> indices = randomSampleIdx();
            problem->estimModelFromSamples(indices);

            double sumSqErr = 0;
            int nbInliers = 0;
            for (int j = 0; j < totalNbSamples; ++j)
            {
                double error = problem->estimErrorForSample(j);
                double score=std::min((error * error)/(thres * thres),1.);
                if (score < 1)
                {
                    sumSqErr += (error * error)/(thres * thres);
                    nbInliers++;
                }
                else
                {
                    sumSqErr += 1.;
                }
            }

#pragma omp critical
            {
                double inliersFraction = (double)(nbInliers) / (double)(problem->getTotalNbSamples());

                if (sumSqErr < this->sumSqErr  )
                {
                    this->inliersFraction = inliersFraction;
                    this->sumSqErr = sumSqErr;
                    this->bestIdxSet = indices;
                }
            }
        }

        problem->estimModelFromSamples(bestIdxSet);
        getInliers(thres);
        //problem->estimModelFromSamples(inliersIdx);
    }
private:
    double sumSqErr;
};



} // namespace robest
#endif // MSAC_H
