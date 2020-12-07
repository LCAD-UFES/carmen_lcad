# Map Memorization and Forgetting in the IARA Autonomous Car 
## [Click here to Access the Technical Report (.pdf)](https://arxiv.org/ftp/arxiv/papers/1810/1810.02355.pdf)

## Abstract: 

In this work, we present a novel strategy to correct imperfections in occupancy grid maps called map decay. The objective of map decay is to correct invalid occupancy probabilities of map cells that are unobservable by sensors. The strategy was inspired by an analogy between the memory architecture believed to exist in the brain and the maps maintained by an autonomous vehicle. It consists in merging sensory information obtained in runtime (online) with a priori data from a high-precision map constructed offline. In map decay, cells observed by sensors are updated using traditional occupancy grid mapping techniques and unobserved cells are adjusted so that their occupancy probabilities tend to the values found in the offline map. This strategy is grounded in the idea that the most precise information available about an unobservable cell is the value found in the high-precision offline map. Map decay was successfully tested and still in use in the IARA autonomous vehicle from Universidade Federal do Espírito Santo. 

## Acknowledgements

Research supported by Conselho Nacional de Desenvolvimento Científico e Tecnológico – CNPq, Brazil (grants 552630/2011-0, 12786/2013-1, 311120/2016-4 and 311504/2017-5), Fundação de Amparo à Pesquisa do Espírito Santo – FAPES, Brazil (grant 48511579/2009), and by Vale/FAPES (grant 75537958/16).
