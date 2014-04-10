#ifndef STATE_INTP_HPP
#define STATE_INTP_HPP

typedef std::vector< double > state_type;
typedef std::vector< double >::iterator iter_1d;
typedef std::vector< state_type >::iterator iter_2d;

// typedef std::vector<double> state_type(3);
// typedef std::vector<double>::iterator iter_1d;

/*!
  Class holds the address of state_types and times vectors.  It uses 
  interpolation to provide the state at a specified time.
*/
class state_intp {
    size_t indx_;
    iter_1d geq_;
    size_t i_;
  
public:
    std::vector< state_type > * m_states;
    std::vector< double > * m_times;
    size_t m_xlen;

    //! \todo Alex: make inputs const ref type
    /*!
      Initializes pointers to user maintained state_type and times vectors.
      \param[in] states Vector of states sampled at different points in time
      \param[in] times Vector of times corresponding to the sampled states
    */
    state_intp( std::vector< state_type > &states , 
		std::vector< double > &times )
	: indx_( 0 ) ,
	  geq_( times.begin() ) ,
	  m_states( &states ) ,
	  m_times( &times )
	{
	    try { m_xlen = states[0].size(); throw size_t(states.size()); }
	    catch( size_t sz ) { 
		std::cout << "Exception initializing m_xlen from states[0].size()"
			  << " in state_intp constructor. States vector is of length: "
			  << sz << "\n";
	    }
	    
	}

    //! \todo Alex: make inputs const ref type
    /*!
      Initializes pointers to user maintained state_type and times vectors.
      \param[in] states Vector of states sampled at different points in time
      \param[in] times Vector of times corresponding to the sampled states
      \param[in] xlength The length of state x(t)
    */
    state_intp( std::vector< state_type > &states , 
		std::vector< double > &times, 
		size_t xlength )
	: indx_( 0 ) ,
	  geq_( times.begin() ) ,
	  m_states( &states ) ,
	  m_times( &times ) ,
	  m_xlen( xlength ) 
	{ }
	// m_states( &states ), 
	// 			   m_times( &times ) , 
	// 			   m_xlen( xlength ) , 
	// 			   indx_( 0 ) , 
	// 			   geq_( times.begin() ) { }

    /*!
      Applies linear interpolation to provide the state at the specified time.
      \param[in] t_intp The time to interpolate the state
      \param[out] x_intp The state at the specified time
    */
    bool operator() (const double t_intp, state_type& x_intp)
	{
	    bool err = false;
	    // find first value in range >= that specifie.d
	    geq_ = std::lower_bound( m_times->begin(), m_times->end(), t_intp );
	    indx_ = geq_-m_times->begin();

	    if ( indx_ == 0 ) {
		err = true;
		x_intp = (*m_states)[indx_];
	    }
	    else if (indx_ == m_times->size()) {
		err = true;
		for (i_=0; i_ < m_xlen; i_++ )
	    	    x_intp[i_] = (*m_states)[indx_-1][i_];
	    }
	    else {  
		for (i_=0; i_ < m_xlen; i_++ ) {
		    x_intp[i_] = (*m_states)[indx_-1][i_] 
			+ ((*m_states)[indx_][i_]-(*m_states)[indx_-1][i_])
			*(t_intp-(*m_times)[indx_-1])/((*m_times)[indx_]
						       -(*m_times)[indx_-1]);
		}
	    }
	    return err;
	}

    //! \todo Alex: make inputs const ref type
    /*!
      Updates the state_type and times vectors pointed to.
      \param[in] states Vector of states sampled at different points in time
      \param[out] times Vector of times corresponding to the sampled states 
    */
    void update( std::vector< state_type > &states , 
		 std::vector< double > &times ) {
	m_states = &states;
	m_times = &times;
    }

    /*!
      \return The first element of the vector of times.
    */
    double begin( ) { return *( m_times->begin( ) ); }

    /*!
      \return The last element of the vector of times.
    */
    double end( ) { return *( m_times->end( )-1 ); }
};

#endif // STATE_INTP_HPP
