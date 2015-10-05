```
```
#
``` visualiktester_april
```
Intro to component here


## Configuration parameters
As any other component,
``` *visualiktester_april* ```
needs a configuration file to start. In

    etc/config

you can find an example of a configuration file. We can find there the following lines:

    EXAMPLE HERE

    
## Starting the component
To avoid changing the *config* file in the repository, we can copy it to the component's home directory, so changes will remain untouched by future git pulls:

    cd

``` <visualiktester_april 's path> ```

    cp etc/config config
    
After editing the new config file we can run the component:

    bin/

```visualiktester_april ```

    --Ice.Config=config
