# 🍓 Fraise 🍓
the control hardware for the bayesian machine accelerator to be integrated in Pinaipple

## Goal of this repo
This repo is a module to encapsulate the bayesian accelerator developped by the INTEGNANO team at C2N. 

## structure of the repo : 
* rtl/ : 
    + low_interface/ a folder with the first layer of interaction with the Memristor array.
    + fraise_top.sv the top rtl file for fraise.  
* fraise.core : a fusesoc description file. 

## list all cores in the repo :

```bash
fusesoc --cores-root=. --log-file fusesoc.log list-cores 
```
