

public interface Num<A implements Num<A>> {
    A add(A that);
    A negate(); // opposite, negative, multiply by -1 addative inverse
    default A sub(A that) {
        return this.add(that.negate());
    }
    A invert(); // multiplicative inverse, (1/this)
    A mul(A that);
    default A div(A that) {
        return this.mul(that.invert());
    }
    A sgn();
    default A sqr() {
        return this.mul(this);
    }
    A sqrt();

}