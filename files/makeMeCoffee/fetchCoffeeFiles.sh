FILES="coffee.3ds human.3ds milk.3ds mug.3ds table.3ds wood.jpg"


for i in $FILES; do
	wget http://ljmanso.com/ursusFiles/$i
done;

