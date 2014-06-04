#include "WQUPC.h"
#include <vector>
#include <cmath>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <ros/ros.h>

using namespace std;

WQUPC::WQUPC() {
	// TODO Auto-generated constructor stub
}

WQUPC::~WQUPC() {
	// TODO Auto-generated destructor stub
}


//Method using WQUPC algorithm to unite different nodes of a graph. Equivalent to union find.
void WQUPC::meltChains(vector<vector<vector<int> > > chains,vector<vector<int> > meltingList){
	int N=chains.size();
	//ROS_INFO("nombre de chaine= %i",N);
	id.clear();

	//initialization of id
	for(int i=0;i<N;i++){
		vector<int> dublet;
		dublet.push_back(i);
		dublet.push_back(chains.at(i).size());
		id.push_back(dublet);
		//ROS_INFO("nb pixels par chaine= %i",chains.at(i).size());
	}

	//makes union vector
	while (!meltingList.empty()){

		vector<int> uni = meltingList.back();
		meltingList.pop_back();

		//ROS_INFO("nd1=%i",uni.at(0));
		//ROS_INFO("nd2=%i",uni.at(1));

		if(!areConnected(uni.at(0),uni.at(1))){
			unite(uni.at(0),uni.at(1));
		}
	}


	//actually does the union with entire chains and not their node
	meltedChains.clear();
	vector<int> rootList;
	for(int j=0;j<id.size();j++){
		//finir de mettre les père à jour
		int useless=root(j);

		//si le noeud n'est pas racine
		if(id.at(j).at(0)!=j){
		//insérer dans le noeud racine
			chains.at( id.at(j).at(0) ).insert(chains.at(id.at(j).at(0)).end() , chains.at(j).begin() , chains.at(j).end() );
		}
		//sinon stocker la racine
		else{rootList.push_back(j);}
	}

	//une fois que ceci est fait, on ne sélectionne que les chaines maximales
	for(int j=0;j<rootList.size();j++){
		meltedChains.push_back(chains.at( rootList.at(j) ));
	}


}
//////////


//Method to know if 2 nodes are connected
bool WQUPC::areConnected(int p,int q){
	 return root(p) == root(q);
}
//////////


//Method to get the root of a node
int WQUPC::root(int i){

	while(i!=id.at(i).at(0)){
		//pour des cas croisés 1fils de2 2fils de 3, bien remettre 1 fils de 3
		id.at(i).at(0)=id.at( (id.at(i).at(0)) ).at(0);
		i=id.at(i).at(0);
	}
	return i;
}
//////////


//Method to unify the subtrees
void WQUPC::unite(int p,int q){
	int i=root(p);
	int j=root(q);

	if(id.at(i).at(1)<id.at(j).at(1)){
		id.at(i).at(0) = j;
		id.at(j).at(1) += id.at(i).at(1);

	}
	else{
		id.at(j).at(0) = i;
		id.at(i).at(1) += id.at(j).at(1);

	}
}
//////////


//Method to send back the various complete chains
vector<vector<vector <int> > > WQUPC::sendBack(){
	return meltedChains;
}
//////////







