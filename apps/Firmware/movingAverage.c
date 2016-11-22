/**
 \file movingAverage.c
 
 \brief Implementación de medias móviles para el cálculo de rssi.
 */

#include "movingAverage.h"

int8_t addDataMovingAverage(uint16_t address, uint8_t nSeq, int8_t rssi, uint8_t time)
{
  uint8_t i = 0;
  uint8_t indexTable = 0;
  bool inArray = false;
  //guardamos el dato en la tabla
  while(i<indexMovingAverage && !inArray)
  {
    if(table[i].address == address)
    {
      //añadimos aquí el dato
      table[i].time = time;
      table[i].datos[table[i].index].nSeq=nSeq;
      table[i].datos[table[i].index].rssi=rssi;
      table[i].index = (table[i].index+1)%WINDOWS_SIZE;
      //return;
      indexTable = i;      
      inArray = true; //salimos del while
     }
     i++;
  }
  if(!inArray && indexMovingAverage<MOVING_AVERAGE_SIZE_TABLE)
  { //no estaba en la tabla, lo metemos
    table[indexMovingAverage].address = address;
    table[indexMovingAverage].time = time;
    table[indexMovingAverage].datos[0].nSeq = nSeq;
    table[indexMovingAverage].datos[0].rssi = rssi;
    table[indexMovingAverage].index = 1;
    
    indexTable = indexMovingAverage;    
    indexMovingAverage++;
    inArray = true;
  }
  
  if (inArray) //hay dato, calculamos la media móvil
  {
    uint8_t lastNSeq = 0;
    int8_t index = 0;
    int8_t origIndex = 0;
    uint8_t j = 0;
    int8_t movingAverage = 0;
    uint8_t fail = 0;
    //sacamos los posibles valores de rssi
    index = table[indexTable].index-1;
    if (index < 0)
      index = WINDOWS_SIZE - 1;
    origIndex = index;
    lastNSeq = table[indexTable].datos[index].nSeq;
    if(!movingAverageWithFour)
    { //con 3
      for (j = 0; j<WINDOWS_SIZE-1; j++)
      {
        if (table[indexTable].datos[index].nSeq == (uint8_t)(lastNSeq-j)) //casting a uint8_t para coger secuencias que dan la vuelta
        {
          //result[j] = table[i].datos[index].rssi;
          if (j == 0) //rssi_ultimo*0.5
            movingAverage +=table[indexTable].datos[index].rssi >> 1;
          else  //+rssi*0.25
            movingAverage += table[indexTable].datos[index].rssi >> 2;
          index--;
        }
        else
        {
          movingAverage += table[indexTable].datos[origIndex].rssi >> 2; //+rssi_ultimo*0.25
          fail++;
        }
        if (index < 0)
          index = WINDOWS_SIZE - 1;
      }
      if (fail == 2)
        return table[indexTable].datos[origIndex].rssi;
      else
        return movingAverage;
    }
    else
    {//con 4      
      for (j = 0; j<WINDOWS_SIZE; j++)
      {
        if (table[indexTable].datos[index].nSeq == (uint8_t)(lastNSeq-j)) //casting a uint8_t para coger secuencias que dan la vuelta
        {
          //result[j] = table[i].datos[index].rssi;
          if (j == 0) //+rssi_ultimo*0.5
            movingAverage += table[indexTable].datos[index].rssi >> 1;
          else
          {
            if (j == 1) //+rssi*0.25
              movingAverage += table[indexTable].datos[index].rssi >> 2;
            else//+rssi*0.125
              movingAverage += table[indexTable].datos[index].rssi >> 3;
          }           
          index--;
        }
        else
        { //nos falta dato
          if (j == 1) //+rssi_ultimo*0.25
            movingAverage += table[indexTable].datos[origIndex].rssi >> 2;
          else //+rssi_ultimo*0.125
            movingAverage += table[indexTable].datos[origIndex].rssi >> 3;
          fail++;
        }
        if (index < 0)
        index = WINDOWS_SIZE - 1;
      }
      if (fail == 3)
        return table[indexTable].datos[origIndex].rssi;
      else
        return movingAverage;
    }        
  }
  return 0;
  
}

void removeDataMovingAverage(uint8_t index)
{
  uint8_t k;
  table[index].address = 0x0;
  table[index].index = 0;
  table[index].time = 0;
  for (k = 0; k < WINDOWS_SIZE; k++)
  {
    table[index].datos[k].nSeq = 0;
    table[index].datos[k].rssi = 0;
  }
}

void clearDataMovingAverage(uint8_t time)
{
  int8_t i;
  uint8_t max = indexMovingAverage-1;
  for (i = max; i >= 0; i--)
  {
    if(table[i].address != 0x0 && (((uint8_t)(table[i].time+1)) < time))
    { 
      uint8_t last = indexMovingAverage-1; //último elemento 
      if (i == last) 
      {
        removeDataMovingAverage(i);
      }
      else
      {
        //si no es el último elemento, vamos desplazando todos los elementos
        uint8_t j;        
        for(j = i; j < last; j++) //¿indexMovingAverage-1?
        {
          uint8_t k;
          //uint8_t next = j+1;
          table[j].address = table[j+1].address;
          table[j].index = table[j+1].index;
          table[j].time = table[j+1].time;
          for(k = 0; k < WINDOWS_SIZE; k++)
          {
            table[j].datos[k].nSeq = table[j+1].datos[k].nSeq;
            table[j].datos[k].rssi = table[j+1].datos[k].rssi;
          }          
        }
        //eliminamos el último elemento   
        removeDataMovingAverage(last);
      }
      //decrementamos el índice
      indexMovingAverage--;
    }
  }
}

void configMovingAverage(bool withFour)
{
  movingAverageWithFour = withFour;
}