## RAM difference composition

This test measures the RAM impact resulting from registering a node as a rclcpp component.

### DATA x86_64

#### 1 Base Node

```
./install/performance_composition/lib/performance_composition/simple_base_main
```

 - VSZ fixed at 618200 KB
 - RSS small variations: 11832, 11948, 11928, 11864, 11824, 11844, 11832, 11852, 11904, 11848

#### 1 Composable Node

```
./install/performance_composition/lib/performance_composition/simple_composable_main
```

 - VSZ fixed at 618436 KB
 - RSS small variations: 12216, 12128, 12240, 12124, 12100, 12132, 12020, 12136, 12040, 12212

