package frc.robot.util;

import java.util.Collection;
import java.util.LinkedList;

/*
 * Java's LinkedList, except removing items from the end after MAX_SIZE is reached.
 * Overrides add, addFirst, addLast, and push methods
 * Overrides addAll methods with unsupported exceptions
 */
public class LimitedStack<T> extends LinkedList<T>{
	
	private static final long serialVersionUID = 1L;
	private int limitedSize;
	
	public LimitedStack(int size) {
		super();
		limitedSize = size;
    }

    //if the max size is reached, remove the oldest item
    private void checkSize(){
        if(this.size() >= limitedSize) {
			this.removeLast();
		}
    }
    
    @Override
    public boolean add(T e) {
        checkSize();
        return super.add(e);
    }

    @Override
    public void addFirst(T e) {
        checkSize();
        super.addFirst(e);
    }

    @Override
    public void addLast(T e) {
        checkSize();
        super.addLast(e);
    }
	
	@Override
	public void push(T e) {
		checkSize();
		super.push(e);
    }

    @Override
    public boolean addAll(Collection<? extends T> c) {
        throw new UnsupportedOperationException();
    }

    @Override
    public boolean addAll(int arg0, Collection<? extends T> arg1) {
        throw new UnsupportedOperationException();
    }
	
}