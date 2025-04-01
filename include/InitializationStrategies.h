/*
 * InitializationStrategies.h
 *
 *  Created on: 27 oct. 2022
 *      Author: pablo
 */

#ifndef INCLUDE_INITIALIZATIONSTRATEGIES_H_
#define INCLUDE_INITIALIZATIONSTRATEGIES_H_


#include <Initializer.h>
#include <InputReader.h>
#include <MapCreationDTO.h>
#include <MapManager.h>

typedef pair<int, int> Match;

class InitializationStrategy{
    public:
    bool initialized;
    int num, f0, f1;
    virtual MapManager* initialize() = 0;
    virtual bool has_Outliers()=0;
    void createInitialMap();
    map<int, float> outliers_prop;

    protected:
    InputReader* mpInputReader;
    MapManager* mpMapManager = new MapManager();
	Initializer* mpInitializer;
    cv::Mat mK;
	cv::Mat Rcw; // Current Camera Rotation
	cv::Mat tcw; // Current Camera Translation
	std::vector<cv::Point3f> mvIniP3D;
	vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)
	vector<cv::KeyPoint> kpsUn1,kpsUn2;
    map<int, Match> mvIniMatches;
    vector<tuple<int,int,int> > mvPairs;
    vector<int> bordes;
};

typedef InitializationStrategy* ( *CreateInitializationStrategyFn)(InputReader* mpInputReader,float dist);

class MostMatches_InitializationStrategy: public InitializationStrategy {

    public:
	MostMatches_InitializationStrategy(InputReader* mpInputReader,float dist);//este es utilizado por la factoría, el parámetro no se usa
	MapManager* initialize();
    bool has_Outliers(){ return false; }
	static InitializationStrategy *  Create(InputReader* mpInputReader,float dist) {
		return new MostMatches_InitializationStrategy(mpInputReader, dist);
	}
};

class EveryPair_InitializationStrategy: public InitializationStrategy {

    public:
	EveryPair_InitializationStrategy(InputReader* mpInputReader,float dist);//este es utilizado por la factoría, el parámetro no se usa
	MapManager* initialize();
    bool has_Outliers(){ return true; }
	static InitializationStrategy *  Create(InputReader* mpInputReader,float dist) {
		return new EveryPair_InitializationStrategy(mpInputReader, dist);
	}
};

class FactoryInitializationStrategies
{
    private:
	FactoryInitializationStrategies();
	FactoryInitializationStrategies(const FactoryInitializationStrategies &) { }
	FactoryInitializationStrategies &operator=(const FactoryInitializationStrategies &) { return *this; }

    typedef std::map<int,CreateInitializationStrategyFn> FactoryMap;
    FactoryMap m_FactoryMap;
    
    public:
    ~FactoryInitializationStrategies() { m_FactoryMap.clear(); }

    static FactoryInitializationStrategies *Get()
    {
        static FactoryInitializationStrategies instance;
        return &instance;
    }

    void Register(const int tipo, CreateInitializationStrategyFn pfnCreate);
    InitializationStrategy *CreateInitializationStrategy(const int type, InputReader* mpInputReader,float dist);
};

#endif /* INCLUDE_INITIALIZATIONSTRATEGIES_H_ */
