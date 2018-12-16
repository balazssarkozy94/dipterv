-- C-RTU GT CR-PM3 base example modul BEGIN

-- module data ( 99 < ccanChannel < 112 )
insert into CRModuleDatas ( moduleTypeId, dataType,ccanChannel,description,tag) VALUES ( 'CR-PM3','VERSION'  ,100,'Bootloader verzió', 'BL_VER');
insert into CRModuleDatas ( moduleTypeId, dataType,ccanChannel,description,tag) VALUES ( 'CR-PM3','VERSION'  ,101,'Bootloader paraméter verzió', 'BL_PARAM_VER');
insert into CRModuleDatas ( moduleTypeId, dataType,ccanChannel,description,tag) VALUES ( 'CR-PM3','VERSION'  ,102,'Program verzió', 'PROG_VER');
insert into CRModuleDatas ( moduleTypeId, dataType,ccanChannel,description,tag) VALUES ( 'CR-PM3','VERSION'  ,103,'Program paraméter verzió', 'PROG_PAR_VER');
insert into CRModuleDatas ( moduleTypeId, dataType,ccanChannel,description,tag) VALUES ( 'CR-PM3','STRING'   ,104,'Program típus', 'PROG_TYPE');
insert into CRModuleDatas ( moduleTypeId, dataType,ccanChannel,description,tag) VALUES ( 'CR-PM3','VERSION'  ,105,'Hardware verzió', 'HW_VER');
insert into CRModuleDatas ( moduleTypeId, dataType,ccanChannel,description,tag) VALUES ( 'CR-PM3','STRING'   ,106,'Hardware típus', 'HW_TYPE');
insert into CRModuleDatas ( moduleTypeId, dataType,ccanChannel,description,tag) VALUES ( 'CR-PM3','STRING'   ,107,'Sorozatszám', 'SERIAL_NUMBER');
insert into CRModuleDatas ( moduleTypeId, dataType,ccanChannel,description,tag) VALUES ( 'CR-PM3','MODULE_HB',109,'Modul életjel', 'MODULE_HB');
insert into CRModuleDatas ( moduleTypeId, dataType,ccanChannel,description,tag) VALUES ( 'CR-PM3','STRING'   ,110,'Utolsó CAN hiba', 'LAST_CAN_ERR');
insert into CRModuleDatas ( moduleTypeId, dataType,ccanChannel,description,tag) VALUES ( 'CR-PM3','COUNTER'  ,111,'Utolsó kapcsolat vesztés oka', 'LAST_CONN_ERR');

--- app data ---
insert into CRModuleDatas ( moduleTypeId, dataType,ccanChannel,description,tag, siUnit) VALUES ('CR-PM3','MEASURE',400,'L1 fázis hőmérséklet', 'L1_TEMP', '°C');
insert into CRModuleDatas ( moduleTypeId, dataType,ccanChannel,description,tag, siUnit) VALUES ('CR-PM3','MEASURE',401,'L1 fázis feszültség', 'L1_VRMS', 'V');
insert into CRModuleDatas ( moduleTypeId, dataType,ccanChannel,description,tag, siUnit) VALUES ('CR-PM3','MEASURE',402,'L1 fázis áram', 'L1_IRMS', 'A');
insert into CRModuleDatas ( moduleTypeId, dataType,ccanChannel,description,tag, siUnit) VALUES ('CR-PM3','MEASURE',403,'L1 fázis teljesítmény', 'L1_POW', 'W');
insert into CRModuleDatas ( moduleTypeId, dataType,ccanChannel,description,tag, siUnit) VALUES ('CR-PM3','MEASURE',404,'L1 fázis energia', 'L1_ENER', 'Wh');

insert into CRModuleDatas ( moduleTypeId, dataType,ccanChannel,description,tag, siUnit) VALUES ('CR-PM3','MEASURE',405,'L2 fázis hőmérséklet', 'L2_TEMP', '°C');
insert into CRModuleDatas ( moduleTypeId, dataType,ccanChannel,description,tag, siUnit) VALUES ('CR-PM3','MEASURE',406,'L2 fázis feszültség', 'L2_VRMS', 'V');
insert into CRModuleDatas ( moduleTypeId, dataType,ccanChannel,description,tag, siUnit) VALUES ('CR-PM3','MEASURE',407,'L2 fázis áram', 'L2_IRMS', 'A');
insert into CRModuleDatas ( moduleTypeId, dataType,ccanChannel,description,tag, siUnit) VALUES ('CR-PM3','MEASURE',408,'L2 fázis teljesítmény', 'L2_POW', 'W');
insert into CRModuleDatas ( moduleTypeId, dataType,ccanChannel,description,tag, siUnit) VALUES ('CR-PM3','MEASURE',409,'L2 fázis energia', 'L2_ENER', 'Wh');

insert into CRModuleDatas ( moduleTypeId, dataType,ccanChannel,description,tag, siUnit) VALUES ('CR-PM3','MEASURE',410,'L3 fázis hőmérséklet', 'L3_TEMP', '°C');
insert into CRModuleDatas ( moduleTypeId, dataType,ccanChannel,description,tag, siUnit) VALUES ('CR-PM3','MEASURE',411,'L3 fázis feszültség', 'L3_VRMS', 'V');
insert into CRModuleDatas ( moduleTypeId, dataType,ccanChannel,description,tag, siUnit) VALUES ('CR-PM3','MEASURE',412,'L3 fázis áram', 'L3_IRMS', 'A');
insert into CRModuleDatas ( moduleTypeId, dataType,ccanChannel,description,tag, siUnit) VALUES ('CR-PM3','MEASURE',413,'L3 fázis teljesítmény', 'L3_POW', 'W');
insert into CRModuleDatas ( moduleTypeId, dataType,ccanChannel,description,tag, siUnit) VALUES ('CR-PM3','MEASURE',414,'L3 fázis energia', 'L3_ENER', 'Wh');

insert into CRModuleDatas ( moduleTypeId, dataType,ccanChannel,description,tag, siUnit) VALUES ('CR-PM3','MEASURE',415,'Nullvezető hőmérséklet', 'N_TEMP', '°C');
insert into CRModuleDatas ( moduleTypeId, dataType,ccanChannel,description,tag, siUnit) VALUES ('CR-PM3','MEASURE',416,'Nullvezető feszültség', 'N_VRMS', 'V');
insert into CRModuleDatas ( moduleTypeId, dataType,ccanChannel,description,tag, siUnit) VALUES ('CR-PM3','MEASURE',417,'Nullvezető áram', 'N_IRMS', 'A');

--- data params ---

--- L1 ---
insert into CRModuleParams (moduleTypeId, ccanChannel, paramId, paramType, paramValue, description, siUnit) VALUES ('CR-PM3', 299,  1, 'uint32', 0,   'L1 fázis hőmérséklet küdési ciklusidő (>0 küld)', 'ms');
insert into CRModuleParams (moduleTypeId, ccanChannel, paramId, paramType, paramValue, description, siUnit) VALUES ('CR-PM3', 299,  2, 'float',  1, 'L1 fázis hőmérséklet küdés szignifikancia (>0 küld)', '');

insert into CRModuleParams (moduleTypeId, ccanChannel, paramId, paramType, paramValue, description, siUnit) VALUES ('CR-PM3', 299,  3, 'uint32', 500,   'L1 fázis feszültség küdési ciklusidő (>0 küld)', 'ms');
insert into CRModuleParams (moduleTypeId, ccanChannel, paramId, paramType, paramValue, description, siUnit) VALUES ('CR-PM3', 299,  4, 'float',  0.1, 'L1 fázis feszültség küdés szignifikancia (>0 küld)', '');

insert into CRModuleParams (moduleTypeId, ccanChannel, paramId, paramType, paramValue, description, siUnit) VALUES ('CR-PM3', 299,  5, 'uint32', 500,   'L1 fázis áram küdési ciklusidő (>0 küld)', 'ms');
insert into CRModuleParams (moduleTypeId, ccanChannel, paramId, paramType, paramValue, description, siUnit) VALUES ('CR-PM3', 299,  6, 'float',  0.1, 'L1 fázis áram küdés szignifikancia (>0 küld)', '');

insert into CRModuleParams (moduleTypeId, ccanChannel, paramId, paramType, paramValue, description, siUnit) VALUES ('CR-PM3', 299,  7, 'uint32', 500,   'L1 fázis teljesítmény küdési ciklusidő (>0 küld)', 'ms');
insert into CRModuleParams (moduleTypeId, ccanChannel, paramId, paramType, paramValue, description, siUnit) VALUES ('CR-PM3', 299,  8, 'float',  0.1, 'L1 fázis teljesítmény küdés szignifikancia (>0 küld)', '');

insert into CRModuleParams (moduleTypeId, ccanChannel, paramId, paramType, paramValue, description, siUnit) VALUES ('CR-PM3', 299,  9, 'uint32', 500,   'L1 fázis energia küdési ciklusidő (>0 küld)', 'ms');
insert into CRModuleParams (moduleTypeId, ccanChannel, paramId, paramType, paramValue, description, siUnit) VALUES ('CR-PM3', 299,  10, 'float',  0.1, 'L1 fázis energia küdés szignifikancia (>0 küld)', '');

--- L2 ---
insert into CRModuleParams (moduleTypeId, ccanChannel, paramId, paramType, paramValue, description, siUnit) VALUES ('CR-PM3', 299,  11, 'uint32', 0,   'L2 fázis hőmérséklet küdési ciklusidő (>0 küld)', 'ms');
insert into CRModuleParams (moduleTypeId, ccanChannel, paramId, paramType, paramValue, description, siUnit) VALUES ('CR-PM3', 299,  12, 'float',  1, 'L2 fázis hőmérséklet küdés szignifikancia (>0 küld)', '');

insert into CRModuleParams (moduleTypeId, ccanChannel, paramId, paramType, paramValue, description, siUnit) VALUES ('CR-PM3', 299,  13, 'uint32', 500,   'L2 fázis feszültség küdési ciklusidő (>0 küld)', 'ms');
insert into CRModuleParams (moduleTypeId, ccanChannel, paramId, paramType, paramValue, description, siUnit) VALUES ('CR-PM3', 299,  14, 'float',  0.1, 'L2 fázis feszültség küdés szignifikancia (>0 küld)', '');

insert into CRModuleParams (moduleTypeId, ccanChannel, paramId, paramType, paramValue, description, siUnit) VALUES ('CR-PM3', 299,  15, 'uint32', 500,   'L2 fázis áram küdési ciklusidő (>0 küld)', 'ms');
insert into CRModuleParams (moduleTypeId, ccanChannel, paramId, paramType, paramValue, description, siUnit) VALUES ('CR-PM3', 299,  16, 'float',  0.1, 'L2 fázis áram küdés szignifikancia (>0 küld)', '');

insert into CRModuleParams (moduleTypeId, ccanChannel, paramId, paramType, paramValue, description, siUnit) VALUES ('CR-PM3', 299,  17, 'uint32', 500,   'L2 fázis teljesítmény küdési ciklusidő (>0 küld)', 'ms');
insert into CRModuleParams (moduleTypeId, ccanChannel, paramId, paramType, paramValue, description, siUnit) VALUES ('CR-PM3', 299,  18, 'float',  0.1, 'L2 fázis teljesítmény küdés szignifikancia (>0 küld)', '');

insert into CRModuleParams (moduleTypeId, ccanChannel, paramId, paramType, paramValue, description, siUnit) VALUES ('CR-PM3', 299,  19, 'uint32', 500,   'L2 fázis energia küdési ciklusidő (>0 küld)', 'ms');
insert into CRModuleParams (moduleTypeId, ccanChannel, paramId, paramType, paramValue, description, siUnit) VALUES ('CR-PM3', 299,  20, 'float',  0.1, 'L2 fázis energia küdés szignifikancia (>0 küld)', '');

--- L3 ---
insert into CRModuleParams (moduleTypeId, ccanChannel, paramId, paramType, paramValue, description, siUnit) VALUES ('CR-PM3', 299,  21, 'uint32', 0,   'L3 fázis hőmérséklet küdési ciklusidő (>0 küld)', 'ms');
insert into CRModuleParams (moduleTypeId, ccanChannel, paramId, paramType, paramValue, description, siUnit) VALUES ('CR-PM3', 299,  22, 'float',  1, 'L3 fázis hőmérséklet küdés szignifikancia (>0 küld)', '');

insert into CRModuleParams (moduleTypeId, ccanChannel, paramId, paramType, paramValue, description, siUnit) VALUES ('CR-PM3', 299,  23, 'uint32', 500,   'L3 fázis feszültség küdési ciklusidő (>0 küld)', 'ms');
insert into CRModuleParams (moduleTypeId, ccanChannel, paramId, paramType, paramValue, description, siUnit) VALUES ('CR-PM3', 299,  24, 'float',  0.1, 'L3 fázis feszültség küdés szignifikancia (>0 küld)', '');

insert into CRModuleParams (moduleTypeId, ccanChannel, paramId, paramType, paramValue, description, siUnit) VALUES ('CR-PM3', 299,  25, 'uint32', 500,   'L3 fázis áram küdési ciklusidő (>0 küld)', 'ms');
insert into CRModuleParams (moduleTypeId, ccanChannel, paramId, paramType, paramValue, description, siUnit) VALUES ('CR-PM3', 299,  26, 'float',  0.1, 'L3 fázis áram küdés szignifikancia (>0 küld)', '');

insert into CRModuleParams (moduleTypeId, ccanChannel, paramId, paramType, paramValue, description, siUnit) VALUES ('CR-PM3', 299,  27, 'uint32', 500,   'L3 fázis teljesítmény küdési ciklusidő (>0 küld)', 'ms');
insert into CRModuleParams (moduleTypeId, ccanChannel, paramId, paramType, paramValue, description, siUnit) VALUES ('CR-PM3', 299,  28, 'float',  0.1, 'L3 fázis teljesítmény küdés szignifikancia (>0 küld)', '');

insert into CRModuleParams (moduleTypeId, ccanChannel, paramId, paramType, paramValue, description, siUnit) VALUES ('CR-PM3', 299,  29, 'uint32', 500,   'L3 fázis energia küdési ciklusidő (>0 küld)', 'ms');
insert into CRModuleParams (moduleTypeId, ccanChannel, paramId, paramType, paramValue, description, siUnit) VALUES ('CR-PM3', 299,  30, 'float',  0.1, 'L3 fázis energia küdés szignifikancia (>0 küld)', '');

--- N ---
insert into CRModuleParams (moduleTypeId, ccanChannel, paramId, paramType, paramValue, description, siUnit) VALUES ('CR-PM3', 299,  31, 'uint32', 0,   'Nullvezető hőmérséklet küdési ciklusidő (>0 küld)', 'ms');
insert into CRModuleParams (moduleTypeId, ccanChannel, paramId, paramType, paramValue, description, siUnit) VALUES ('CR-PM3', 299,  32, 'float',  1, 'Nullvezető hőmérséklet küdés szignifikancia (>0 küld)', '');

insert into CRModuleParams (moduleTypeId, ccanChannel, paramId, paramType, paramValue, description, siUnit) VALUES ('CR-PM3', 299,  33, 'uint32', 500,   'Nullvezető feszültség küdési ciklusidő (>0 küld)', 'ms');
insert into CRModuleParams (moduleTypeId, ccanChannel, paramId, paramType, paramValue, description, siUnit) VALUES ('CR-PM3', 299,  34, 'float',  0.1, 'Nullvezető feszültség küdés szignifikancia (>0 küld)', '');

insert into CRModuleParams (moduleTypeId, ccanChannel, paramId, paramType, paramValue, description, siUnit) VALUES ('CR-PM3', 299,  35, 'uint32', 500,   'Nullvezető áram küdési ciklusidő (>0 küld)', 'ms');
insert into CRModuleParams (moduleTypeId, ccanChannel, paramId, paramType, paramValue, description, siUnit) VALUES ('CR-PM3', 299,  36, 'float',  0.1, 'Nullvezető áram küdés szignifikancia (>0 küld)', '');

--- app params ---


-- C-RTU GT CR-PM3 modul END
--------------------------------------------------------------------------------------------------------------
