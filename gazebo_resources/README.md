# gazebo_resources

Copy models to local gazebo directory (hidden ```.gazebo``` in local ```$HOME``` folder) 
```
cp -r models/* $HOME/.gazebo/models/
```

The folder ```model_facets``` contains a ```.csv``` file with the facet centroid coordinates and normal orientations corresponding to the 3D mesh model of the boat used for the project. The helper script ```facet_viewpoints_extractor.py``` used to extract these is also provided in the same folder.