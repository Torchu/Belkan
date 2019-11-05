#include "../Comportamientos_Jugador/jugador.hpp"
#include "motorlib/util.h"

#include <iostream>
#include <cmath>
#include <set>
#include <stack>
#include <queue>


// Este es el método principal que debe contener los 4 Comportamientos_Jugador
// que se piden en la práctica. Tiene como entrada la información de los
// sensores y devuelve la acción a realizar.
Action ComportamientoJugador::think(Sensores sensores) {
	//Capturar valores de fila y columna
	if(sensores.mensajeF != -1){
		fil = sensores.mensajeF;
		col = sensores.mensajeC;
		ultimaAccion = actIDLE;
		situado = true;
	}

	//Actualizar el efecto de la última acción
	switch(ultimaAccion){
		case actTURN_R: brujula = (brujula+1)%4; break;
		case actTURN_L: brujula = (brujula+3)%4; break;
		case actFORWARD:
			switch(brujula){
				case 0: fil--; break;
				case 1: col++; break;
				case 2: fil++; break;
				case 3: col--; break;
			}
			break;
	}

	//Actualizar el mapa (solo nivel 4)
	if(sensores.nivel == 4 && situado)
		actualizarMapa(mapaResultado, sensores);

	//Mirar si ha cambiado el destino
	if(sensores.destinoF != destino.fila || sensores.destinoC != destino.columna){
		destino.fila = sensores.destinoF;
		destino.columna = sensores.destinoC;
		hayPlan = false;
	}

	//Calcular un camino hacia el destino
	if(!hayPlan && situado){
		actual.fila = fil;
		actual.columna = col;
		actual.orientacion = brujula;
		hayPlan = pathFinding(sensores.nivel, actual, destino, plan);
	}

	//Contador de pasos en la misma direccion
	static int counter = 0;

	//Sistema de movimiento
	Action sigAccion;

	//Si hay un plan, se sigue el plan
	if(hayPlan && plan.size()>0){
		sigAccion = plan.front();
		plan.erase(plan.begin());
		if((sensores.terreno[2] == 'P' || sensores.terreno[2] == 'M' || sensores.terreno[2] == 'D' || sensores.superficie[2] == 'a') && sigAccion == actFORWARD){
			sigAccion = actIDLE;
			hayPlan = false;
		}
	}

	//Si no hay plan, se pasa a un comportamiento reactivo
	else{
		hayPlan = false;
		//Si se encuentra un obstaculo gira a la derecha
		if(sensores.terreno[2] == 'P' || sensores.terreno[2] == 'M' || sensores.terreno[2] == 'D' || sensores.superficie[2] == 'a')
			sigAccion = actTURN_R;
		else
			sigAccion = actFORWARD;
		//Cada 200 pasos hace un giro
		counter ++;
		if(counter == 200){
			sigAccion = actTURN_R;
			counter = 0;
		}
		//Busca casillas PK (solo nivel 4)
		if(!situado && sensores.nivel == 4)
			for(int i = 1; i < 16 && !hayPlan; i++)
				if(sensores.terreno[i] == 'K'){
					hayPlan = pathFinding_PK(i, plan);
					sigAccion = actIDLE;
				}
	}

	//Recordar la última acción
	ultimaAccion = sigAccion;

	return sigAccion;
}

//PathFinding para casillas amarillas cuando alguna aparece en nuestra visión
bool ComportamientoJugador::pathFinding_PK(int pos, list<Action> &plan){
	//Borro la lista
	cout << "Calculando plan\n";
	plan.clear();
	int x, y;
	switch(pos){
		case 1:	y = 1; x = -1; break;
		case 2:	y = 1; x =  0; break;
		case 3:	y = 1; x =  1; break;
		case 4:	y = 2; x = -2; break;
		case 5:	y = 2; x = -1; break;
		case 6:	y = 2; x =  0; break;
		case 7:	y = 2; x =  1; break;
		case 8:	y = 2; x =  2; break;
		case 9:	y = 3; x = -3; break;
		case 10:y = 3; x = -2; break;	
		case 11:y = 3; x = -1; break;	
		case 12:y = 3; x =  0; break;
		case 13:y = 3; x =  1; break;	
		case 14:y = 3; x =  2; break;	
		case 15:y = 3; x =  3; break;
	}
	//Se avanzan las casillas necesarias (empieza en 1 para compensar el actFORWRAD que hace)
	for(int i = 0; i < y; i++)
		plan.push_back(actFORWARD);
	//Se consulta si ir a izquierda o a derecha
	if(x < 0)
		plan.push_back(actTURN_L);
	else if (x > 0)
		plan.push_back(actTURN_R);
	//Se avanzan las casillas necesarias
	for(int i = 0; i < abs(x); i++)
		plan.push_back(actFORWARD);

    cout << "Terminada la busqueda\n";
		cout << "Cargando el plan\n";
		cout << "Longitud del plan: " << plan.size() << endl;
		PintaPlan(plan);
		return true;
}
	

// Llama al algoritmo de busqueda que se usará en cada comportamiento del agente
// Level representa el comportamiento en el que fue iniciado el agente.
bool ComportamientoJugador::pathFinding (int level, const estado &origen, const estado &destino, list<Action> &plan){
	switch (level){
		case 1: cout << "Busqueda en profundad\n";
			      return pathFinding_Profundidad(origen,destino,plan);
						break;
		case 2: cout << "Busqueda en Anchura\n";
			      return pathFinding_Anchura(origen,destino,plan);
						break;
		case 3: cout << "Busqueda Costo Uniforme\n";
			      return pathFinding_CosteUniforme(origen,destino,plan);
						break;
		case 4: cout << "Busqueda para el reto\n";
				  return pathFinding_AEstrella(origen, destino, plan);
						break;
	}
	cout << "Comportamiento sin implementar\n";
	return false;
}

//Actualiza el mapa
void ComportamientoJugador::actualizarMapa(std::vector<vector<unsigned char>> &mapa, Sensores sensores){
	switch(brujula){
		case 0:	mapa[fil  ][col  ] = sensores.terreno[0];
				mapa[fil-1][col-1] = sensores.terreno[1];
				mapa[fil-1][col  ] = sensores.terreno[2];
				mapa[fil-1][col+1] = sensores.terreno[3];
				mapa[fil-2][col-2] = sensores.terreno[4];
				mapa[fil-2][col-1] = sensores.terreno[5];
				mapa[fil-2][col  ] = sensores.terreno[6];
				mapa[fil-2][col+1] = sensores.terreno[7];
				mapa[fil-2][col+2] = sensores.terreno[8];
				mapa[fil-3][col-3] = sensores.terreno[9];
				mapa[fil-3][col-2] = sensores.terreno[10];
				mapa[fil-3][col-1] = sensores.terreno[11];
				mapa[fil-3][col  ] = sensores.terreno[12];
				mapa[fil-3][col+1] = sensores.terreno[13];
				mapa[fil-3][col+2] = sensores.terreno[14];
				mapa[fil-3][col+3] = sensores.terreno[15];
				break;
		case 1:	mapa[fil  ][col  ] = sensores.terreno[0];
				mapa[fil-1][col+1] = sensores.terreno[1];
				mapa[fil  ][col+1] = sensores.terreno[2];
				mapa[fil+1][col+1] = sensores.terreno[3];
				mapa[fil-2][col+2] = sensores.terreno[4];
				mapa[fil-1][col+2] = sensores.terreno[5];
				mapa[fil  ][col+2] = sensores.terreno[6];
				mapa[fil+1][col+2] = sensores.terreno[7];
				mapa[fil+2][col+2] = sensores.terreno[8];
				mapa[fil-3][col+3] = sensores.terreno[9];
				mapa[fil-2][col+3] = sensores.terreno[10];
				mapa[fil-1][col+3] = sensores.terreno[11];
				mapa[fil  ][col+3] = sensores.terreno[12];
				mapa[fil+1][col+3] = sensores.terreno[13];
				mapa[fil+2][col+3] = sensores.terreno[14];
				mapa[fil+3][col+3] = sensores.terreno[15];
				break;
		case 2:	mapa[fil  ][col  ] = sensores.terreno[0];
				mapa[fil+1][col+1] = sensores.terreno[1];
				mapa[fil+1][col  ] = sensores.terreno[2];
				mapa[fil+1][col-1] = sensores.terreno[3];
				mapa[fil+2][col+2] = sensores.terreno[4];
				mapa[fil+2][col+1] = sensores.terreno[5];
				mapa[fil+2][col  ] = sensores.terreno[6];
				mapa[fil+2][col-1] = sensores.terreno[7];
				mapa[fil+2][col-2] = sensores.terreno[8];
				mapa[fil+3][col+3] = sensores.terreno[9];
				mapa[fil+3][col+2] = sensores.terreno[10];
				mapa[fil+3][col+1] = sensores.terreno[11];
				mapa[fil+3][col  ] = sensores.terreno[12];
				mapa[fil+3][col-1] = sensores.terreno[13];
				mapa[fil+3][col-2] = sensores.terreno[14];
				mapa[fil+3][col-3] = sensores.terreno[15];
				break;
		case 3:	mapa[fil  ][col  ] = sensores.terreno[0];
				mapa[fil+1][col-1] = sensores.terreno[1];
				mapa[fil  ][col-1] = sensores.terreno[2];
				mapa[fil-1][col-1] = sensores.terreno[3];
				mapa[fil+2][col-2] = sensores.terreno[4];
				mapa[fil+1][col-2] = sensores.terreno[5];
				mapa[fil  ][col-2] = sensores.terreno[6];
				mapa[fil-1][col-2] = sensores.terreno[7];
				mapa[fil-2][col-2] = sensores.terreno[8];
				mapa[fil+3][col-3] = sensores.terreno[9];
				mapa[fil+2][col-3] = sensores.terreno[10];
				mapa[fil+1][col-3] = sensores.terreno[11];
				mapa[fil  ][col-3] = sensores.terreno[12];
				mapa[fil-1][col-3] = sensores.terreno[13];
				mapa[fil-2][col-3] = sensores.terreno[14];
				mapa[fil-3][col-3] = sensores.terreno[15];
				break;
	}
}


//---------------------- Implementación de la busqueda en profundidad ---------------------------

// Dado el código en carácter de una casilla del mapa dice si se puede
// pasar por ella sin riegos de morir o chocar.
bool EsObstaculo(unsigned char casilla){
	if (casilla=='P' or casilla=='M' or casilla =='D')
		return true;
	else
	  return false;
}


// Comprueba si la casilla que hay delante es un obstaculo. Si es un
// obstaculo devuelve true. Si no es un obstaculo, devuelve false y
// modifica st con la posición de la casilla del avance.
bool ComportamientoJugador::HayObstaculoDelante(estado &st){
	int fil=st.fila, col=st.columna;

  // calculo cual es la casilla de delante del agente
	switch (st.orientacion) {
		case 0: fil--; break;
		case 1: col++; break;
		case 2: fil++; break;
		case 3: col--; break;
	}

	// Compruebo que no me salgo fuera del rango del mapa
	if (fil<0 or fil>=mapaResultado.size()) return true;
	if (col<0 or col>=mapaResultado[0].size()) return true;

	// Miro si en esa casilla hay un obstaculo infranqueable
	if (!EsObstaculo(mapaResultado[fil][col])){
		// No hay obstaculo, actualizo el parámetro st poniendo la casilla de delante.
    	st.fila = fil;
		st.columna = col;
		return false;
	}
	else{
	  return true;
	}
}




struct nodo{
	estado st;
	list<Action> secuencia;
	int coste;
	int prioridad;
	bool operator<(const nodo &n) const{
		return (this->prioridad > n.prioridad);
	}
};

struct ComparaEstados{
	bool operator()(const estado &a, const estado &n) const{
		if ((a.fila > n.fila) or (a.fila == n.fila and a.columna > n.columna) or
	      (a.fila == n.fila and a.columna == n.columna and a.orientacion > n.orientacion))
			return true;
		else
			return false;
	}
};


// Implementación de la búsqueda en profundidad.
// Entran los puntos origen y destino y devuelve la
// secuencia de acciones en plan, una lista de acciones.
bool ComportamientoJugador::pathFinding_Profundidad(const estado &origen, const estado &destino, list<Action> &plan) {
	//Borro la lista
	cout << "Calculando plan\n";
	plan.clear();
	set<estado,ComparaEstados> generados; 	// Lista de Cerrados
	stack<nodo> pila;						// Lista de Abiertos

  nodo current;
	current.st = origen;
	current.secuencia.empty();

	pila.push(current);

  while (!pila.empty() and (current.st.fila!=destino.fila or current.st.columna != destino.columna)){

		pila.pop();
		generados.insert(current.st);

		// Generar descendiente de girar a la derecha
		nodo hijoTurnR = current;
		hijoTurnR.st.orientacion = (hijoTurnR.st.orientacion+1)%4;
		if (generados.find(hijoTurnR.st) == generados.end()){
			hijoTurnR.secuencia.push_back(actTURN_R);
			pila.push(hijoTurnR);

		}

		// Generar descendiente de girar a la izquierda
		nodo hijoTurnL = current;
		hijoTurnL.st.orientacion = (hijoTurnL.st.orientacion+3)%4;
		if (generados.find(hijoTurnL.st) == generados.end()){
			hijoTurnL.secuencia.push_back(actTURN_L);
			pila.push(hijoTurnL);
		}

		// Generar descendiente de avanzar
		nodo hijoForward = current;
		if (!HayObstaculoDelante(hijoForward.st)){
			if (generados.find(hijoForward.st) == generados.end()){
				hijoForward.secuencia.push_back(actFORWARD);
				pila.push(hijoForward);
			}
		}

		// Tomo el siguiente valor de la pila
		if (!pila.empty()){
			current = pila.top();
		}
	}

  cout << "Terminada la busqueda\n";

	if (current.st.fila == destino.fila and current.st.columna == destino.columna){
		cout << "Cargando el plan\n";
		plan = current.secuencia;
		cout << "Longitud del plan: " << plan.size() << endl;
		PintaPlan(plan);
		// ver el plan en el mapa
		VisualizaPlan(origen, plan);
		return true;
	}
	else {
		cout << "No encontrado plan\n";
	}


	return false;
}

// Sacar por la términal la secuencia del plan obtenido
void ComportamientoJugador::PintaPlan(list<Action> plan) {
	auto it = plan.begin();
	while (it!=plan.end()){
		if (*it == actFORWARD){
			cout << "A ";
		}
		else if (*it == actTURN_R){
			cout << "D ";
		}
		else if (*it == actTURN_L){
			cout << "I ";
		}
		else {
			cout << "- ";
		}
		it++;
	}
	cout << endl;
}

void AnularMatriz(vector<vector<unsigned char> > &m){
	for (int i=0; i<m[0].size(); i++){
		for (int j=0; j<m.size(); j++){
			m[i][j]=0;
		}
	}
}

// Pinta sobre el mapa del juego el plan obtenido
void ComportamientoJugador::VisualizaPlan(const estado &st, const list<Action> &plan){
  AnularMatriz(mapaConPlan);
	estado cst = st;

	auto it = plan.begin();
	while (it!=plan.end()){
		if (*it == actFORWARD){
			switch (cst.orientacion) {
				case 0: cst.fila--; break;
				case 1: cst.columna++; break;
				case 2: cst.fila++; break;
				case 3: cst.columna--; break;
			}
			mapaConPlan[cst.fila][cst.columna]=1;
		}
		else if (*it == actTURN_R){
			cst.orientacion = (cst.orientacion+1)%4;
		}
		else {
			cst.orientacion = (cst.orientacion+3)%4;
		}
		it++;
	}
}

int ComportamientoJugador::interact(Action accion, int valor){
  return false;
}

//---------------------- Implementación de la busqueda en anchura ---------------------------
bool ComportamientoJugador::pathFinding_Anchura(const estado &origen, const estado &destino, list<Action> &plan) {
	//Borro la lista
	cout << "Calculando plan\n";
	plan.clear();
	set<estado,ComparaEstados> generados; 	// Lista de Cerrados
	queue<nodo> cola;						// Lista de Abiertos

  nodo current;
	current.st = origen;
	current.secuencia.empty();

	cola.push(current);

  while (!cola.empty() and (current.st.fila!=destino.fila or current.st.columna != destino.columna)){

		cola.pop();
		generados.insert(current.st);

		// Generar descendiente de girar a la derecha
		nodo hijoTurnR = current;
		hijoTurnR.st.orientacion = (hijoTurnR.st.orientacion+1)%4;
		if (generados.find(hijoTurnR.st) == generados.end()){
			hijoTurnR.secuencia.push_back(actTURN_R);
			cola.push(hijoTurnR);

		}

		// Generar descendiente de girar a la izquierda
		nodo hijoTurnL = current;
		hijoTurnL.st.orientacion = (hijoTurnL.st.orientacion+3)%4;
		if (generados.find(hijoTurnL.st) == generados.end()){
			hijoTurnL.secuencia.push_back(actTURN_L);
			cola.push(hijoTurnL);
		}

		// Generar descendiente de avanzar
		nodo hijoForward = current;
		if (!HayObstaculoDelante(hijoForward.st)){
			if (generados.find(hijoForward.st) == generados.end()){
				hijoForward.secuencia.push_back(actFORWARD);
				cola.push(hijoForward);
			}
		}

		// Tomo el siguiente valor de la cola
		if (!cola.empty()){
			current = cola.front();
		}
	}

  cout << "Terminada la busqueda\n";

	if (current.st.fila == destino.fila and current.st.columna == destino.columna){
		cout << "Cargando el plan\n";
		plan = current.secuencia;
		cout << "Longitud del plan: " << plan.size() << endl;
		PintaPlan(plan);
		// ver el plan en el mapa
		VisualizaPlan(origen, plan);
		return true;
	}
	else {
		cout << "No encontrado plan\n";
	}


	return false;
}

//---------------------- Implementación de la busqueda con costo uniforme ---------------------------
int calculaCoste(unsigned char tipoCasilla){
	int coste;
	switch(tipoCasilla){
		case 'B': coste = 5; break;
		case 'A': coste = 10; break;
		case 'T': coste = 2; break;
		case '?': coste = 3; break;
		default: coste = 1;
	}
	return coste;
}

bool ComportamientoJugador::pathFinding_CosteUniforme(const estado &origen, const estado &destino, list<Action> &plan) {
	//Borro la lista
	cout << "Calculando plan\n";
	plan.clear();
	set<estado,ComparaEstados> generados; 	// Lista de Cerrados
	priority_queue<nodo> cola;				// Lista de Abiertos

  nodo current;
	current.st = origen;
	current.secuencia.empty();
	current.coste = calculaCoste(mapaResultado[origen.fila][origen.columna]);
	current.prioridad = current.coste;

	cola.push(current);

  while (!cola.empty() and (current.st.fila!=destino.fila or current.st.columna != destino.columna)){

		cola.pop();
		generados.insert(current.st);

		// Generar descendiente de girar a la derecha
		nodo hijoTurnR = current;
		hijoTurnR.st.orientacion = (hijoTurnR.st.orientacion+1)%4;
		if (generados.find(hijoTurnR.st) == generados.end()){
			hijoTurnR.secuencia.push_back(actTURN_R);
			hijoTurnR.coste ++;
			hijoTurnR.prioridad ++;
			cola.push(hijoTurnR);

		}

		// Generar descendiente de girar a la izquierda
		nodo hijoTurnL = current;
		hijoTurnL.st.orientacion = (hijoTurnL.st.orientacion+3)%4;
		if (generados.find(hijoTurnL.st) == generados.end()){
			hijoTurnL.secuencia.push_back(actTURN_L);
			hijoTurnL.coste ++;
			hijoTurnL.prioridad ++;
			cola.push(hijoTurnL);
		}

		// Generar descendiente de avanzar
		nodo hijoForward = current;
		if (!HayObstaculoDelante(hijoForward.st)){
			if (generados.find(hijoForward.st) == generados.end()){
				hijoForward.secuencia.push_back(actFORWARD);
				hijoForward.coste += calculaCoste(mapaResultado[hijoForward.st.fila][hijoForward.st.columna]);
				hijoForward.prioridad = hijoForward.coste;
				cola.push(hijoForward);
			}
		}

		// Tomo el siguiente valor de la cola
		if (!cola.empty()){
			current = cola.top();
		}
	}

  cout << "Terminada la busqueda\n";

	if (current.st.fila == destino.fila and current.st.columna == destino.columna){
		cout << "Cargando el plan\n";
		plan = current.secuencia;
		cout << "Longitud del plan: " << plan.size() << endl;
		PintaPlan(plan);
		// ver el plan en el mapa
		VisualizaPlan(origen, plan);
		return true;
	}
	else {
		cout << "No encontrado plan\n";
	}

	return false;
}

//---------------------- Implementación de la busqueda A* ---------------------------
int calculaManhattan(const estado &origen, const estado &destino){
	return abs(origen.fila - destino.fila) + abs(origen.columna - destino.columna);
}

bool ComportamientoJugador::pathFinding_AEstrella(const estado &origen, const estado &destino, list<Action> &plan) {
	//Borro la lista
	cout << "Calculando plan\n";
	plan.clear();
	set<estado,ComparaEstados> generados; 	// Lista de Cerrados
	priority_queue<nodo> cola;				// Lista de Abiertos

  nodo current;
	current.st = origen;
	current.secuencia.empty();
	current.coste = calculaCoste(mapaResultado[origen.fila][origen.columna]);
	current.prioridad = current.coste + calculaManhattan(current.st, destino);

	cola.push(current);

	while (!cola.empty() and (current.st.fila!=destino.fila or current.st.columna != destino.columna)){

		cola.pop();
		generados.insert(current.st);

		// Generar descendiente de girar a la derecha
		nodo hijoTurnR = current;
		hijoTurnR.st.orientacion = (hijoTurnR.st.orientacion+1)%4;
		if (generados.find(hijoTurnR.st) == generados.end()){
			hijoTurnR.secuencia.push_back(actTURN_R);
			hijoTurnR.coste ++;
			hijoTurnR.prioridad ++;
			cola.push(hijoTurnR);

		}

		// Generar descendiente de girar a la izquierda
		nodo hijoTurnL = current;
		hijoTurnL.st.orientacion = (hijoTurnL.st.orientacion+3)%4;
		if (generados.find(hijoTurnL.st) == generados.end()){
			hijoTurnL.secuencia.push_back(actTURN_L);
			hijoTurnL.coste ++;
			hijoTurnL.prioridad ++;
			cola.push(hijoTurnL);
		}

		// Generar descendiente de avanzar
		nodo hijoForward = current;
		if (!HayObstaculoDelante(hijoForward.st)){
			if (generados.find(hijoForward.st) == generados.end()){
				hijoForward.secuencia.push_back(actFORWARD);
				hijoForward.coste += calculaCoste(mapaResultado[hijoForward.st.fila][hijoForward.st.columna]);
				hijoForward.prioridad = hijoForward.coste + calculaManhattan(hijoForward.st, destino);
				cola.push(hijoForward);
			}
		}

		// Tomo el siguiente valor de la cola
		if (!cola.empty()){
			current = cola.top();
		}
	}

  cout << "Terminada la busqueda\n";

	if (current.st.fila == destino.fila and current.st.columna == destino.columna){
		cout << "Cargando el plan\n";
		plan = current.secuencia;
		cout << "Longitud del plan: " << plan.size() << endl;
		PintaPlan(plan);
		// ver el plan en el mapa
		VisualizaPlan(origen, plan);
		return true;
	}
	else {
		cout << "No encontrado plan\n";
	}

	return false;
}