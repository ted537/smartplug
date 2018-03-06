typedef struct {
	// basically a bool
	int switchon;
	// plop data in here
} pininputs;

void adjustwithtemp();
void adjustwithouttemp();
pininputs readpins();

int main() {
	// currently goes once
	// gotta make it into some sorta loop
	pininputs inputs = readpins();
	if (inputs.switchon) {
		adjustwithtemp();	
	}
	else {
		adjustwithouttemp();
	}
}

pininputs readpins() {

}

void adjustwithtemp() {

}

void adjustwithouttemp() {

}
