#!binbash

echo 'This script will use docker build to build the Agent Example with the latest tag and push it to the waraps registry.'

read -p Are you sure [Yn] -n 1 -r
echo    # (optional) move to a new line
if [[ $REPLY =~ ^[Yy]$ ]]   
then
    echo Script Starting
    docker build -t python_agent:latest .
    docker tag python_agent:latest registry.waraps.org/python_agent:latest
    docker push registry.waraps.org/python_agent:latest
fi

echo Script done 👌 BYE