```mermaid
graph LR;
  pick((pick))-->dock((Dock))
  dock-->return((Return))
  return-->undock((Undock))
  undock-->home((Home))
  dock-->place((Place))
  place-->return
  place-->place
  pick-->pick
  undock-->pick
  home-->pick
```
